#include "driver/gpio.h"
#include "soc/gpio_periph.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "esp_log.h"

#include "main.h"
#include "comms.h"
#include "PID.h"
#include "storage_flash.h"
#include "mpu6050_wrapper.h"
#include "wifi_handler.h"

#ifdef HARDWARE_PROTOTYPE
    #include "stepper.h"
#endif

/* Incluyo componentes */
#include "tcp_socket_server.h"

#if defined(HARDWARE_HOVERROBOT)
    #include "CAN_MCB.h"
#endif

extern QueueHandle_t mpu6050QueueHandler;                   // Recibo nuevos angulos obtenidos del MPU
QueueHandle_t motorControlQueueHandler;                     // Envio nuevos valores de salida para el control de motores
QueueHandle_t newPidParamsQueueHandler;                     // Recibo nuevos parametros relacionados al pid
QueueHandle_t newCommandQueueHandler;
QueueHandle_t receiveControlQueueHandler;
QueueHandle_t newMcbQueueHandler;
QueueHandle_t socketConnectionStateQueueHandler;
QueueHandle_t networkStateQueueHandler;

TaskHandle_t imuTaskHandler;

static status_robot_t statusRobot;                            // Estructura que contiene todos los parametros de status a enviar a la app
static output_motors_t speedMotors;
static output_motors_t attitudeControlMotor;

static void taskCleanWheels(void *pvParameters);

struct {
    uint8_t attMode;
    float   setPointPosCms; 
    float   setPointSpeed;
    float   setPointYaw;
    uint8_t contSafetyMaxSpeed;
} attitudeControlStat = {
    .attMode = ATT_MODE_ATTI,
    .setPointPosCms = 0.00,
    .setPointSpeed = 0.00,
    .setPointYaw = 0.00,
};

float pos2mts(int32_t steps) {
    return (steps/STEPS_PER_REV) * DIST_PER_REV;
}

/*
  * Calculo de distancia angular para el yaw, donde hay una discontinuidad entre -180 y 180, ya que en realidad ese salto no es tal.
*/
float angularDistance(float setPoint,float actualValue) {

    float error = setPoint - actualValue;

    if(error > 180.00) {
        error -= 360.00;
    }
    else if(error < -180.00) {
        error += 360.00;
    }
    return error + setPoint;
}

float cutAngle(float angleInput) {
    if (angleInput > 180.00) {
        angleInput -= 360;
    }
    else if (angleInput < -180.00) {
        angleInput += 360;
    }
    return angleInput;
}

int16_t cutSpeedRange(int16_t speed) {
    if (speed > 1000) {
        return 1000;
    }
    else if (speed < -1000) {
        return -1000;
    }
    else {
        return speed;
    }
}

int16_t backlashAttenuator(int16_t speed) {
    const uint16_t backlash = 15;
    if(speed > 0 && speed < backlash) {
        speed = backlash;
    }
    else if (speed < 0 && speed > -backlash) {
        speed = -backlash;
    }
    return speed;
}

void setStatusRobot(uint8_t newStatus) {
    const char *TAG = "StatusRobot";
    
    if (statusRobot.statusCode != newStatus) {
        switch(newStatus) {

            case STATUS_ROBOT_INIT:
            case STATUS_ROBOT_TEST_MODE:
            break;

            case STATUS_ROBOT_STABILIZED:
                pidClearTerms(PID_ANGLE);
                pidClearTerms(PID_SPEED);
                statusRobot.speedL = 0;
                statusRobot.speedR = 0;
                speedMotors.motorL = 0;
                speedMotors.motorR = 0;

                attitudeControlStat.attMode = ATT_MODE_ATTI;
                attitudeControlMotor.motorL = 0;
                attitudeControlMotor.motorR = 0;
                speedMotors.enable = true;
                ESP_LOGI(TAG,"ROBOT STABILIZED");
            break;

            case STATUS_ROBOT_ARMED:
                pidSetDisable(PID_ANGLE);
                speedMotors.enable = false;
                speedMotors.motorL = 0;
                speedMotors.motorR = 0;
                statusRobot.localConfig.pids[PID_ANGLE].setPoint = 0;
                pidSetSetPoint(PID_ANGLE,statusRobot.localConfig.pids[PID_ANGLE].setPoint);

                attitudeControlStat.attMode = ATT_MODE_ATTI;
                ESP_LOGI(TAG,"DISABLED -> ROBOT ARMED, safetyLimits: %f",statusRobot.localConfig.safetyLimits);
            break;

            case STATUS_ROBOT_ERROR:
            case STATUS_ROBOT_ERROR_BATTERY:
            case STATUS_ROBOT_ERROR_HALL_L:
            case STATUS_ROBOT_ERROR_HALL_R:
            case STATUS_ROBOT_ERROR_IMU:
            case STATUS_ROBOT_ERROR_TEMP:
            case STATUS_ROBOT_ERROR_LIMIT_SPEED:
            case STATUS_ROBOT_ERROR_MCB_CONNECTION:
                speedMotors.enable = false;
                speedMotors.motorL = 0;
                speedMotors.motorR = 0;
                attitudeControlStat.contSafetyMaxSpeed = 0;
                ESP_LOGI(TAG,"ROBOT ERROR: %d",newStatus);
            break;

            default:
                ESP_LOGE(TAG,"Unknown state");
            break;
        }

        updateStatusLed(newStatus);
        statusRobot.statusCode = newStatus;
    }
}

static void errorMcbHandler(status_code_mcb_t statusCode) {
    
    switch (statusCode) {
        case NO_ERROR_MCB:
            if (statusRobot.statusCode == STATUS_ROBOT_ERROR_BATTERY ||
                statusRobot.statusCode == STATUS_ROBOT_ERROR_HALL_L ||
                statusRobot.statusCode == STATUS_ROBOT_ERROR_HALL_R) {
                setStatusRobot(STATUS_ROBOT_ARMED);
                }
        break;

        case ERROR_MCB_BATTERY:
            setStatusRobot(STATUS_ROBOT_ERROR_BATTERY);
        break;

        case ERROR_MCB_TEMP:
            setStatusRobot(STATUS_ROBOT_ERROR_TEMP);
        break;

        case ERROR_MCB_HALL_L:
            #ifdef INVERT_HALL_SIDE
                setStatusRobot(STATUS_ROBOT_ERROR_HALL_R);
            #elif
                setStatusRobot(STATUS_ROBOT_ERROR_HALL_L);
            #endif
        break;

        case ERROR_MCB_HALL_R:
            #ifdef INVERT_HALL_SIDE
                setStatusRobot(STATUS_ROBOT_ERROR_HALL_L);
            #elif
                setStatusRobot(STATUS_ROBOT_ERROR_HALL_R);
            #endif
        break;

        case ERROR_MCB_INACTIVITY:
        break;
    }
}

static void imuControlHandler(void *pvParameters) {
    vector_queue_t newAngles;
    float safetyLimitProm[5];
    uint8_t safetyLimitPromIndex = 0;

    const char *TAG = "ImuControlHandler";

    ESP_LOGI(TAG,"Init imuControlTask");

    while(1) {
        if(xQueueReceive(mpu6050QueueHandler,&newAngles,pdMS_TO_TICKS(10))) {

            statusRobot.actualRoll = newAngles.angles[ANGLE_ROLL];
            statusRobot.actualPitch = newAngles.angles[ANGLE_PITCH];
            statusRobot.actualYaw = newAngles.angles[ANGLE_YAW];
            statusRobot.tempImu = (uint16_t)newAngles.temp;

            
            int16_t meanSpeedMeas = (statusRobot.speedMeasR - statusRobot.speedMeasL) / 20.00;       // velocidad maxima deberia ser 1000
            float desiredAngleControl = (float)(pidCalculate(PID_SPEED, meanSpeedMeas) * MAX_ANGLE_CONTROL); 

            statusRobot.localConfig.pids[PID_ANGLE].setPoint = desiredAngleControl;
            pidSetSetPoint(PID_ANGLE, desiredAngleControl);

            int16_t outputPidMotors = (uint16_t)(pidCalculate(PID_ANGLE,statusRobot.actualPitch) * MAX_VELOCITY); 

            speedMotors.motorL = cutSpeedRange(outputPidMotors + attitudeControlMotor.motorL) * DIRECTION_L_MOTOR;
            speedMotors.motorR = cutSpeedRange(outputPidMotors + attitudeControlMotor.motorR) * DIRECTION_R_MOTOR;

            #ifdef HARDWARE_HOVERROBOT
                speedMotors.motorL = backlashAttenuator(speedMotors.motorL);
                speedMotors.motorR = backlashAttenuator(speedMotors.motorR);
            #endif

            safetyLimitProm[safetyLimitPromIndex++] = statusRobot.actualPitch;
            if (safetyLimitPromIndex > 2) {
                safetyLimitPromIndex = 0;
            }
            float angleSafetyLimit = (safetyLimitProm[0] + safetyLimitProm[1] + safetyLimitProm[2]) / 3;

            if (pidGetEnable(PID_ANGLE)) { 
                if ((angleSafetyLimit < (-statusRobot.localConfig.safetyLimits)) ||
                    (angleSafetyLimit > (statusRobot.localConfig.safetyLimits))) { 
                    setStatusRobot(STATUS_ROBOT_ERROR_LIMIT_ANGLE);
                    vTaskDelay(30);
                    setStatusRobot(STATUS_ROBOT_ARMED);
                }
            }
            else { 
                if ((statusRobot.actualPitch > (-MIN_PITCH_ARMED)) && 
                    (statusRobot.actualPitch < (MIN_PITCH_ARMED)) &&
                    (statusRobot.actualRoll > (-MIN_ROLL_ARMED)) && 
                    (statusRobot.actualRoll < (MIN_ROLL_ARMED)) &&
                    statusRobot.statusCode == STATUS_ROBOT_ARMED) { 
                    
                    #ifndef THROTTLE_HOLD_MODE
                        statusRobot.localConfig.pids[PID_ANGLE].setPoint = 0;
                        statusRobot.localConfig.pids[PID_SPEED].setPoint = 0.00; // Inicio con el setpoint de velocidad en 0
                        pidSetSetPoint(PID_ANGLE,statusRobot.localConfig.pids[PID_ANGLE].setPoint);    
                        pidSetSetPoint(PID_SPEED,statusRobot.localConfig.pids[PID_SPEED].setPoint);
                        setStatusRobot(STATUS_ROBOT_STABILIZED);
                        pidSetEnable(PID_ANGLE);  
                        pidSetEnable(PID_SPEED);
                    #endif
                }
            }

            if (abs(speedMotors.motorR) == 1000 || abs(speedMotors.motorL) == 1000) {
                attitudeControlStat.contSafetyMaxSpeed++;
                if (attitudeControlStat.contSafetyMaxSpeed > MAX_CYCLES_LIMIT_SPEED ) {
                    setStatusRobot(STATUS_ROBOT_ERROR_LIMIT_SPEED);
                    vTaskDelay(30);
                    setStatusRobot(STATUS_ROBOT_ARMED);
                }
            }
            else {
                attitudeControlStat.contSafetyMaxSpeed = 0;
            }
            
            statusRobot.speedL = speedMotors.motorL;
            statusRobot.speedR = speedMotors.motorR;
        }
    }
}

static void attitudeControl(void *pvParameters){
    float desiredSpeedControl = 0.00;
    uint8_t isYawControlEnabled = false;

    const char *TAG = "AttitudeControlTask";

    while(true) {

        if (statusRobot.statusCode == STATUS_ROBOT_STABILIZED) {

            if (!statusRobot.dirControl.joyAxisX) {     // Yaw control

                if (!isYawControlEnabled) { 
                    attitudeControlStat.setPointYaw = statusRobot.actualYaw;
                    statusRobot.localConfig.pids[PID_YAW].setPoint = attitudeControlStat.setPointYaw;
                    pidSetSetPoint(PID_YAW, attitudeControlStat.setPointYaw / 1.8);
                    pidSetEnable(PID_YAW);
                    isYawControlEnabled = true;
                    ESP_LOGI(TAG,"Enable YAW_CONTROL, sp: %f",attitudeControlStat.setPointYaw);
                }

                float angularDist = angularDistance(attitudeControlStat.setPointYaw,statusRobot.actualYaw);
                statusRobot.outputYawControl = pidCalculate(PID_YAW,angularDist / 1.8) * -1;
                attitudeControlMotor.motorR = statusRobot.outputYawControl * MAX_ROTATION_RATE_CONTROL;
                attitudeControlMotor.motorL = attitudeControlMotor.motorR * -1;
            }
            else {
                isYawControlEnabled = false;
                pidSetDisable(PID_YAW);
                // Yaw manual control
                attitudeControlMotor.motorR = (statusRobot.dirControl.joyAxisX / 100.00) * MAX_ROTATION_RATE_CONTROL;
                attitudeControlMotor.motorL = attitudeControlMotor.motorR * -1;
            }

            if (!statusRobot.dirControl.joyAxisY) {     // Pos control
                statusRobot.actualDistInCms = ((statusRobot.posInMetersL + statusRobot.posInMetersR) / 2) * 100.00;

                if (attitudeControlStat.attMode != ATT_MODE_POS_CONTROL) {
                    attitudeControlStat.setPointPosCms = statusRobot.actualDistInCms;           // TODO: podria setearlo cuando llegue a 0 la velocidad de los motores
                    statusRobot.localConfig.pids[PID_POS].setPoint = statusRobot.actualDistInCms;
                    pidSetSetPoint(PID_POS, statusRobot.actualDistInCms);
                    pidSetEnable(PID_POS);
                    attitudeControlStat.attMode = ATT_MODE_POS_CONTROL;
                    ESP_LOGI(TAG,"Enable POS_CONTROL");
                }
         
                desiredSpeedControl = pidCalculate(PID_POS, statusRobot.actualDistInCms) * (MAX_VELOCITY_SPEED_CONTROL/10.00); 
            }
            else {
                desiredSpeedControl = (statusRobot.dirControl.joyAxisY * MAX_VELOCITY_SPEED_CONTROL) / 1000.00;
                if (attitudeControlStat.attMode != ATT_MODE_MANUAL_CONTROL) {
                    pidSetDisable(PID_POS);
                    statusRobot.localConfig.pids[PID_POS].setPoint = 0.00;
                    attitudeControlStat.attMode = ATT_MODE_MANUAL_CONTROL;
                    ESP_LOGI(TAG,"Enable MANUAL_CONTROL");
                }
            }

            statusRobot.localConfig.pids[PID_SPEED].setPoint = desiredSpeedControl;
            pidSetSetPoint(PID_SPEED, desiredSpeedControl);
        }
        else {
            if (isYawControlEnabled) {
                isYawControlEnabled = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void commsManager(void *pvParameters) {
    uint8_t contMcbTimeout = 0, socketClientsConnected = 0, lastSocketClientsConnected = 0;
    bool networkState = false, lastNetworkState = false;
    pid_settings_comms_t    newPidSettings;
    command_app_raw_t       newCommand;
    control_app_raw_t       newControl;

    #ifdef HARDWARE_HOVERROBOT
        rx_motor_control_board_t receiveMcb;
    #endif

    uint8_t toggle = false;
    const char *TAG = "commsManager";

    while(true) {
        if (xQueueReceive(receiveControlQueueHandler,&newControl,0)) {
            statusRobot.dirControl.joyAxisX = newControl.axisX;
            statusRobot.dirControl.joyAxisY = newControl.axisY;
        }
        
        if (xQueueReceive(newPidParamsQueueHandler,&newPidSettings,0)) {
            pidSetConstants(newPidSettings.indexPid,newPidSettings.kp,newPidSettings.ki,newPidSettings.kd);
            if (newPidSettings.indexPid == PID_ANGLE) {
                pidSetSetPoint(PID_ANGLE,newPidSettings.centerAngle);
                statusRobot.localConfig.pids[newPidSettings.indexPid].setPoint = newPidSettings.centerAngle;      

                // statusRobot.localConfig.centerAngle = newPidSettings.centerAngle; 
            }           
            statusRobot.localConfig.pids[newPidSettings.indexPid].kp = newPidSettings.kp;
            statusRobot.localConfig.pids[newPidSettings.indexPid].ki = newPidSettings.ki;
            statusRobot.localConfig.pids[newPidSettings.indexPid].kd = newPidSettings.kd;
                 
            ESP_LOGI(TAG, "\n**\tNuevos parametros %d:\t**\n*\tP: %.2f\t\t\t*\n*\tI: %.2f\t\t\t*\n*\tD: %.2f\t\t\t*\n*\tcenter: %.2f\t\t*\n*\tsafety limits: %.2f\t*\n",newPidSettings.indexPid,newPidSettings.kp,newPidSettings.ki,newPidSettings.kd,newPidSettings.centerAngle,newPidSettings.safetyLimits);              
        }

        if (xQueueReceive(newCommandQueueHandler,&newCommand,0)) {

            switch (newCommand.command) {
                case COMMAND_CALIBRATE_IMU:
                    ESP_LOGI(TAG,"Calibrando IMU...");
                    mpu6050_recalibrate();
                break;
                case COMMAND_CLEAN_WHEELS:
                    if(statusRobot.statusCode == STATUS_ROBOT_ARMED) {
                        const char *msg = (newCommand.value == 0) ? "Izq" : "Der";
                        ESP_LOGI(TAG,"Limpiando rueda %s",msg);
                        xTaskCreate(taskCleanWheels,"clean wheels",2048,&newCommand.value,4,NULL);
                    } else {
                        ESP_LOGI(TAG,"Error status armed");
                    }
                break;
                case COMMAND_SAVE_LOCAL_CONFIG:
                    ESP_LOGI(TAG,"Guardando parametros...");
                    storageLocalConfig(statusRobot.localConfig);
                    sendLocalConfig(statusRobot.localConfig);
                break;

                case COMMAND_MOVE_FORWARD:
                    ESP_LOGI(TAG,"Move forward command, distance: %f",newCommand.value / PRECISION_DECIMALS_COMMS);
                    attitudeControlStat.setPointPosCms += newCommand.value;
                    statusRobot.localConfig.pids[PID_POS].setPoint = attitudeControlStat.setPointPosCms;
                    pidSetSetPoint(PID_POS,attitudeControlStat.setPointPosCms);
                break;

                case COMMAND_MOVE_BACKWARD:
                    ESP_LOGI(TAG,"Move backward command, distance: %f",newCommand.value / PRECISION_DECIMALS_COMMS);
                    attitudeControlStat.setPointPosCms -= newCommand.value;
                    statusRobot.localConfig.pids[PID_POS].setPoint = attitudeControlStat.setPointPosCms;
                    pidSetSetPoint(PID_POS,attitudeControlStat.setPointPosCms);
                break;

                case COMMAND_MOVE_ABS_YAW:
                    float yawAngle = (uint16_t)newCommand.value / PRECISION_DECIMALS_COMMS;
                    ESP_LOGI(TAG,"Move absolute angle: %f, commandValue: %d",yawAngle,newCommand.value);
                    attitudeControlStat.setPointYaw = yawAngle;
                    statusRobot.localConfig.pids[PID_YAW].setPoint = attitudeControlStat.setPointYaw;
                    pidSetSetPoint(PID_YAW, attitudeControlStat.setPointYaw / 1.8);
                break;

                case COMMAND_MOVE_REL_YAW:
                    float newYawAngle = (newCommand.value / PRECISION_DECIMALS_COMMS) + attitudeControlStat.setPointYaw;
                    newYawAngle = cutAngle(newYawAngle);
                    ESP_LOGI(TAG,"Move relative angle: actual: %f,\t relative: %f, \t result: %f",statusRobot.actualYaw,(newCommand.value / PRECISION_DECIMALS_COMMS),newYawAngle);    
                    attitudeControlStat.setPointYaw = newYawAngle;
                    statusRobot.localConfig.pids[PID_YAW].setPoint = attitudeControlStat.setPointYaw;
                    pidSetSetPoint(PID_YAW, attitudeControlStat.setPointYaw / 1.8);
                break;

                case COMMAND_DEARMED_ROBOT:
                    setStatusRobot(STATUS_ROBOT_ARMED);
                    vTaskSuspend(imuTaskHandler);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    vTaskResume(imuTaskHandler);
                break;
            }            
        }
        
        #ifdef HARDWARE_HOVERROBOT
            if(xQueueReceive(newMcbQueueHandler,&receiveMcb,0)) {
                contMcbTimeout = 0;
                if (!statusRobot.isMcbConnected) {
                    statusRobot.isMcbConnected = true;
                    if (statusRobot.statusCode == STATUS_ROBOT_ERROR_MCB_CONNECTION) {
                        setStatusRobot(STATUS_ROBOT_ARMED);
                    }
                }
                #ifdef HARDWARE_MAINBOARD
                    statusRobot.isCharging = receiveMcb.isCharging;
                    statusRobot.tempMcb = receiveMcb.boardTemp / 10.00;
                    errorMcbHandler(receiveMcb.statusCode);
                #endif
                
                statusRobot.batVoltage = receiveMcb.batVoltage;
                statusRobot.speedMeasR = receiveMcb.speedR_meas;
                statusRobot.speedMeasL = receiveMcb.speedL_meas;
                statusRobot.currentR = receiveMcb.currentR;
                statusRobot.currentL = receiveMcb.currentL;
                statusRobot.posInMetersR = pos2mts(receiveMcb.posR);
                statusRobot.posInMetersL = pos2mts(receiveMcb.posL * -1);
            }
        #elif defined(HARDWARE_PROTOTYPE)
            motors_measurements_t newMeasureMotors = getMeasMotors();
            statusRobot.speedMeasR = newMeasureMotors.speedMotR;
            statusRobot.speedMeasL = newMeasureMotors.speedMotL;
            statusRobot.posInMetersR = pos2mts(newMeasureMotors.absPosR);
            statusRobot.posInMetersL = pos2mts(newMeasureMotors.absPosL);
        #endif

        xQueuePeek(networkStateQueueHandler, &networkState, 0);
        if (networkState != lastNetworkState) {
            ESP_LOGI(TAG, "Nuevo estado de conexion wifi: %d", networkState);
            lastNetworkState = networkState;
        }

        xQueuePeek(socketConnectionStateQueueHandler, &socketClientsConnected,0);       // Leo el ultimo valor emitido, sin sacarlo de la queue
        if (networkState && socketClientsConnected > 0) {
            if (socketClientsConnected > lastSocketClientsConnected) {
                sendLocalConfig(statusRobot.localConfig);
                ESP_LOGE(TAG,"Envio nuevo local config");
                
            }
            lastSocketClientsConnected = socketClientsConnected;

            robot_dynamic_data_t newData = {
                .isCharging = statusRobot.isCharging,
                .batVoltage = statusRobot.batVoltage,
                .imuTemp = statusRobot.tempImu * PRECISION_DECIMALS_COMMS,
                .mcbTemp = statusRobot.tempMcb * PRECISION_DECIMALS_COMMS,      // Ya esta multiplicada por 1000 desde la mcb
                .mainboardTemp = statusRobot.tempMainboard,
                .speedR = CONVERT_RPM_TO_MPS(statusRobot.speedMeasR) * PRECISION_DECIMALS_COMMS,
                .speedL = CONVERT_RPM_TO_MPS(statusRobot.speedMeasL) * PRECISION_DECIMALS_COMMS,
                .currentR = statusRobot.currentR,                               // Ya esta multiplicada por 100 desde la MCB
                .currentL = statusRobot.currentL,                               // Ya esta multiplicada por 100 desde la MCB
                .pitch =  statusRobot.actualPitch * PRECISION_DECIMALS_COMMS,
                .roll = statusRobot.actualRoll * PRECISION_DECIMALS_COMMS,
                .yaw = statusRobot.actualYaw * PRECISION_DECIMALS_COMMS,
                .posInMeters = ((statusRobot.posInMetersL + statusRobot.posInMetersR) / 2) * PRECISION_DECIMALS_COMMS,
                .outputYawControl = statusRobot.outputYawControl * PRECISION_DECIMALS_COMMS,
                .setPointAngle = statusRobot.localConfig.pids[PID_ANGLE].setPoint * PRECISION_DECIMALS_COMMS,
                .setPointPos = statusRobot.localConfig.pids[PID_POS].setPoint,                                          // No lo multiplico, para mandarlo en mts
                .setPointYaw = statusRobot.localConfig.pids[PID_YAW].setPoint * PRECISION_DECIMALS_COMMS,
                .setPointSpeed = CONVERT_RPM_TO_MPS(statusRobot.localConfig.pids[PID_SPEED].setPoint) * PRECISION_DECIMALS_COMMS * 10,      // El setpoint en el PID es RPM/10
                .centerAngle = statusRobot.localConfig.centerAngle * PRECISION_DECIMALS_COMMS,
                .statusCode = statusRobot.statusCode
            };
            sendDynamicData(newData);
        }

        xQueueSend(motorControlQueueHandler,&speedMotors,0);

        // gpio_set_level(PIN_OSCILO, toggle);
        toggle = !toggle;

        contMcbTimeout++;
        if(contMcbTimeout > MAX_CONT_TIMEOUT_MCB) {
            statusRobot.isMcbConnected = false;
            setStatusRobot(STATUS_ROBOT_ERROR_MCB_CONNECTION);
        }

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void testHardwareVibration(void) {
    uint8_t flagInc=true;
    int16_t testMotor=0;

    setStatusRobot(STATUS_ROBOT_TEST_MODE);
    vTaskSuspend(imuTaskHandler);


    while(true) {
        printf(">angle:%f\n>outputMotor:%f\n",statusRobot.actualPitch/10.0,statusRobot.speedL/100.0);

        if(flagInc){
            testMotor++;
            if(testMotor>99){
                flagInc=false;
            }
        }
        else {
            testMotor--;
            if(testMotor < -99){
                flagInc=true;
                // testMotor = 0;
            }
        }

        statusRobot.speedL = testMotor*10;
        statusRobot.speedR = testMotor*10;
        speedMotors.motorL = testMotor*3;
        speedMotors.motorR = testMotor*3;
        xQueueSend(motorControlQueueHandler,&speedMotors,pdMS_TO_TICKS(1));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void taskCleanWheels(void *pvParameters) {
    uint16_t testMotor=0;
    uint8_t wheel = *(uint8_t *)pvParameters;

    setStatusRobot(STATUS_ROBOT_TEST_MODE);
    vTaskSuspend(imuTaskHandler);
    
    speedMotors.enable = true;
    for(testMotor = 0;testMotor < SPEED_CLEAN_WHEELS_MS;testMotor+=(SPEED_CLEAN_WHEELS_MS/10)) {

        if (wheel == 0) {
            statusRobot.speedL = testMotor;
            speedMotors.motorL = testMotor;
        } else {
            statusRobot.speedR = testMotor;
            speedMotors.motorR = testMotor;
        }
        vTaskDelay(25);
    }

    vTaskDelay(pdMS_TO_TICKS(TIME_CLEAN_WHEELS_MS));
    statusRobot.speedL = 0;
    statusRobot.speedR = 0;
    speedMotors.motorL = 0;
    speedMotors.motorR = 0;
    speedMotors.enable = false;
    vTaskDelay(pdMS_TO_TICKS(TIME_CLEAN_WHEELS_MS));

    vTaskResume(imuTaskHandler);
    setStatusRobot(STATUS_ROBOT_ARMED);
    vTaskDelete(NULL);
}

void app_main() {
    const char *TAG = "app_main";
    // gpio_set_direction(PIN_LED , GPIO_MODE_OUTPUT);
    // gpio_set_level(PIN_LED, 1);

    // PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_OSCILO], PIN_FUNC_GPIO);
    // gpio_set_direction(PIN_OSCILO , GPIO_MODE_OUTPUT);
    // gpio_set_level(PIN_OSCILO, 1);

    receiveControlQueueHandler = xQueueCreate(1, sizeof(control_app_raw_t));
    newPidParamsQueueHandler = xQueueCreate(1, sizeof(pid_settings_comms_t));
    newCommandQueueHandler = xQueueCreate(1,  sizeof(command_app_raw_t));
    motorControlQueueHandler = xQueueCreate(1, sizeof(output_motors_t));
    mpu6050QueueHandler = xQueueCreate(1, sizeof(vector_queue_t));
    socketConnectionStateQueueHandler = xQueueCreate(1, sizeof(uint8_t));
    networkStateQueueHandler = xQueueCreate(1, sizeof(bool));
    #ifdef HARDWARE_HOVERROBOT
        newMcbQueueHandler = xQueueCreate(1,sizeof(rx_motor_control_board_t));
    #endif
    
    xTaskCreate(statusLedHandler,"status led handler",2048,socketConnectionStateQueueHandler,2,NULL);
    setStatusRobot(STATUS_ROBOT_INIT);

    #ifdef HARDWARE_HOVERROBOT
        statusRobot.localConfig.pids[PID_ANGLE].kp = 0.57;
        statusRobot.localConfig.pids[PID_ANGLE].ki = 0.13;
        statusRobot.localConfig.pids[PID_ANGLE].kd = 1.16;

        statusRobot.localConfig.pids[PID_SPEED].kp = 5.20;
        statusRobot.localConfig.pids[PID_SPEED].ki = 0.14;
        statusRobot.localConfig.pids[PID_SPEED].kd = 0.50;

        statusRobot.localConfig.pids[PID_POS].kp = 0.36;
        statusRobot.localConfig.pids[PID_POS].ki = 0.02;
        statusRobot.localConfig.pids[PID_POS].kd = 0.05;

        statusRobot.localConfig.pids[PID_YAW].kp = 5.00;//2.00;
        statusRobot.localConfig.pids[PID_YAW].ki = 0.5;//0.3;
        statusRobot.localConfig.pids[PID_YAW].kd = 3.00;//0.00;

        statusRobot.localConfig.centerAngle = 0;
        statusRobot.localConfig.safetyLimits = 60;//45;
    #endif

    #ifdef HARDWARE_PROTOTYPE
        statusRobot.localConfig.pids[PID_ANGLE].kp = 1.48;
        statusRobot.localConfig.pids[PID_ANGLE].ki = 0.52;
        statusRobot.localConfig.pids[PID_ANGLE].kd = 0.21;

        //TODO: ajustar parametros
        statusRobot.localConfig.pids[PID_POS].kp = 2.0;
        statusRobot.localConfig.pids[PID_POS].ki = 0.1;
        statusRobot.localConfig.pids[PID_POS].kd = 2.73;

        statusRobot.localConfig.pids[PID_YAW].kp = 2.00;
        statusRobot.localConfig.pids[PID_YAW].ki = 0.3;
        statusRobot.localConfig.pids[PID_YAW].kd = 0.00;

        statusRobot.localConfig.pids[PID_SPEED].kp = 2.80;
        statusRobot.localConfig.pids[PID_SPEED].ki = 0.41;
        statusRobot.localConfig.pids[PID_SPEED].kd = 0.04;

        statusRobot.localConfig.centerAngle = 4.9;
        statusRobot.localConfig.safetyLimits = 45; // 35;
    #endif

    statusRobot.localConfig.pids[PID_ANGLE].setPoint = 0;
    statusRobot.localConfig.pids[PID_SPEED].setPoint = 0;

    ESP_LOGI(TAG, "\n------------------- local config -------------------"); 
    ESP_LOGI(TAG, "safetyLimits: %f\tcenterAngle: %f",statusRobot.localConfig.safetyLimits,statusRobot.localConfig.centerAngle);
    
    for (uint8_t i=0;i<CANT_PIDS;i++) {
        ESP_LOGI(TAG,"PID %d Params: kp: %f\tki: %f\tkd: %f\tsetPoint: %f",i,statusRobot.localConfig.pids[i].kp,statusRobot.localConfig.pids[i].ki,statusRobot.localConfig.pids[i].kd,statusRobot.localConfig.pids[i].setPoint);
    }
    ESP_LOGI(TAG, "\n------------------- local config -------------------\n"); 
    
    mpu6050_init_t configMpu = {
        .intGpio = GPIO_MPU_INT,
        .sclGpio = GPIO_MPU_SCL,
        .sdaGpio = GPIO_MPU_SDA,
        .priorityTask = MPU_HANDLER_PRIORITY,
        .core = IMU_HANDLER_CORE
    };
    // mpu6050_initialize(&configMpu);

    pid_init_t pidConfig;
    pidConfig.sampleTimeInMs = PERIOD_IMU_MS;
    memcpy(pidConfig.pids,statusRobot.localConfig.pids,sizeof(pidConfig.pids));
    pidInit(pidConfig);

    #ifdef HARDWARE_HOVERROBOT
        config_init_mcb_t configMcb = {
            .numUart = UART_PORT_CAN,
            .txPin = GPIO_CAN_TX,
            .rxPin = GPIO_CAN_RX,
            .queue = newMcbQueueHandler,
            .core = 0
        };
        mcbInit(&configMcb);
    #endif

    #ifdef HARDWARE_PROTOTYPE
        stepper_config_t configMotors = {
            .gpio_mot_l_step = GPIO_MOT_L_STEP,
            .gpio_mot_l_dir = GPIO_MOT_L_DIR,
            .gpio_mot_r_step = GPIO_MOT_R_STEP,
            .gpio_mot_r_dir = GPIO_MOT_R_DIR,
            .gpio_mot_enable = GPIO_MOT_ENABLE,
            .gpio_mot_microstepper = GPIO_MOT_MICRO_STEP
        };
        motorsInit(configMotors);
        setMicroSteps(true);
    #endif

    // esp_log_level_set("wifi", ESP_LOG_WARN);
    // esp_log_level_set("wifi_init", ESP_LOG_WARN);
    #ifdef NETWORK_WIFI_MODE_AP
        initWifi(ESP_WIFI_SSID, ESP_WIFI_PASS, WIFI_MODE_AP, networkStateQueueHandler);
    #else
        initWifi(ESP_WIFI_SSID, ESP_WIFI_PASS, WIFI_MODE_STA, networkStateQueueHandler);
    #endif
    
    // initTcpClientSocket(appConnectionStateQueueHandler);
    initTcpServerSocket(socketConnectionStateQueueHandler);

    setStatusRobot(STATUS_ROBOT_ARMED);
    xTaskCreatePinnedToCore(imuControlHandler,"Imu Control",4096,NULL,IMU_HANDLER_PRIORITY,&imuTaskHandler,IMU_HANDLER_CORE);
    xTaskCreatePinnedToCore(attitudeControl,"attitude control",4096,NULL,4,NULL,IMU_HANDLER_CORE);
    xTaskCreatePinnedToCore(commsManager,"communication manager",4096,NULL,COMM_HANDLER_PRIORITY,NULL,IMU_HANDLER_CORE);;

    // temperature_sensor_config_t temp_sensor_config = {
    //     .range_min = 0,
    //     .range_max = 100,
    //     .clk_src = APB_CLK_FREQ
    // };

    // temperature_sensor_handle_t temp_handle = NULL;
    // // temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 50);
    // ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
    
    // while (true) { 
    // // Enable temperature sensor
    // ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
    // // Get converted sensor data
    // float tsens_out;
    // ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));
    // printf("Temperature in %f °C\n", tsens_out);
    // // Disable the temperature sensor if it is not needed and save the power
    // ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));
    // }
}
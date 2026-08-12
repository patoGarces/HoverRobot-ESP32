#include "driver/gpio.h"
#include "soc/gpio_periph.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"
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
#include "ultrasonic.h"
#include "udp_logger.h"
#include "drive_controller.h"

/* Incluyo componentes */
#include "tcp_socket_component.h"
#include "nav_comms.h"

extern QueueHandle_t mpu6050QueueHandler;                   // Recibo nuevos angulos obtenidos del MPU
QueueHandle_t driveControllerMotorQueue;                     // Envio nuevos valores de salida para el control de motores
QueueHandle_t driveControllerDataReceiveQueue;
QueueHandle_t newPidParamsQueueHandler;                     // Recibo nuevos parametros relacionados al pid
QueueHandle_t newCommandQueueHandler;
QueueHandle_t receiveControlQueueHandler;
static QueueHandle_t socketConnectionStateQueueHandler;
QueueHandle_t networkStateQueueHandler;
QueueHandle_t collisionSensorsQueue;

extern StreamBufferHandle_t xStreamBufferReceiver;
extern StreamBufferHandle_t xStreamBufferSender;

TaskHandle_t imuTaskHandler;

static status_robot_t statusRobot;                            // Estructura que contiene todos los parametros de status a enviar a la app
static drive_controller_motor_control_t speedMotors;                // TODO: hace falta definir 2? un speedMotors y otro attitudeControlMotors?
static drive_controller_motor_control_t attitudeControlMotor;

static void taskCleanWheels(void *pvParameters);
static void PidAngleStepResponseTask(void *pvParameters);

struct {
    uint8_t attMode;
    float   setPointPosCms; 
    float   setPointSpeed;
    float   setPointYaw;
    float   offsetDistInCms;
    uint8_t contSafetyMaxSpeed;
} attitudeControlStat = {
    .attMode = ATT_MODE_ATTI,
    .setPointPosCms = 0.00,
    .setPointSpeed = 0.00,
    .setPointYaw = 0.00,
    .offsetDistInCms = 0.00
};

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
                statusRobot.speedTargetL = 0;
                statusRobot.speedTargetR = 0;
                speedMotors.motorL = 0;
                speedMotors.motorR = 0;

                attitudeControlStat.offsetDistInCms = (statusRobot.posInMetersL + statusRobot.posInMetersR) / 2.00;
                statusRobot.actualDistInCms = 0.00;

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

        #ifdef HARDWARE_MAINBOARD
            updateStatusLed(newStatus);
        #endif
        statusRobot.statusCode = newStatus;
    }
}

static void driveControllerStatusHandler(drive_controller_status_code_t statusCode) {
    
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
            #else
                setStatusRobot(STATUS_ROBOT_ERROR_HALL_L);
            #endif
        break;

        case ERROR_MCB_HALL_R:
            #ifdef INVERT_HALL_SIDE
                setStatusRobot(STATUS_ROBOT_ERROR_HALL_L);
            #else
                setStatusRobot(STATUS_ROBOT_ERROR_HALL_R);
            #endif
        break;

        case ERROR_MCB_INACTIVITY:
        break;
    }
}

static void imuControlHandler(void *pvParameters) {
    vector_queue_t newAngles;
    float safetyLimitProm[5], imuYaw = 0.00, encYawTheta = 0.00, encYawDeg = 0.00, lastPosR = 0.00, lastPosL = 0.00;
    uint8_t contMcbTimeout = 0, safetyLimitPromIndex = 0;
    drive_controller_data_t newMotorDataReceived;
    const uint8_t driveControllerTicksTimeout = TIMEOUT_MCB_MS / 5.0;                   

    while(1) {
        if(xQueueReceive(mpu6050QueueHandler,&newAngles,pdMS_TO_TICKS(10))) {

            statusRobot.actualRoll = newAngles.angles[ANGLE_ROLL];
            statusRobot.actualPitch = newAngles.angles[ANGLE_PITCH];
            imuYaw = newAngles.angles[ANGLE_YAW];
            statusRobot.tempImu = newAngles.temp;

            encYawTheta += ((statusRobot.posInMetersR - lastPosR) - (statusRobot.posInMetersL-lastPosL)) / WHEEL_BASE;       
            
            lastPosL = statusRobot.posInMetersL;
            lastPosR = statusRobot.posInMetersR;

            // Convierto a grados para fusionar con imuYaw
            encYawDeg = encYawTheta * (180.0f / M_PI);

            statusRobot.actualYaw = FUSE_ALPHA_YAW * imuYaw + (1.00 - FUSE_ALPHA_YAW) * encYawDeg;

            // ESP_LOGI("calc yaw", "encYawDeg: %f\timuYaw: %f\tresult: %f", encYawDeg, imuYaw, statusRobot.actualYaw);

            pidSetSetPoint(PID_ANGLE, statusRobot.localConfig.pids[PID_ANGLE].setPoint);

            int16_t outputPidMotors = (int16_t)(pidCalculate(PID_ANGLE,statusRobot.actualPitch) * MAX_VELOCITY_RPM); 

            speedMotors.motorL = cutSpeedRange(outputPidMotors + attitudeControlMotor.motorL) * DIRECTION_L_MOTOR;
            speedMotors.motorR = cutSpeedRange(outputPidMotors + attitudeControlMotor.motorR) * DIRECTION_R_MOTOR;

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
            
            statusRobot.speedTargetL = speedMotors.motorL;
            statusRobot.speedTargetR = speedMotors.motorR;

            // gpio_set_level(PIN_OSCILO, toggle);
            // toggle = !toggle;
        }


        if (xQueueReceive(driveControllerDataReceiveQueue, &newMotorDataReceived, 0)) {
            contMcbTimeout = 0;
            if (!statusRobot.isMcbConnected) {
                statusRobot.isMcbConnected = true;
                if (statusRobot.statusCode == STATUS_ROBOT_ERROR_MCB_CONNECTION) {
                    setStatusRobot(STATUS_ROBOT_ARMED);
                }
            }

            // TODO: paso intermedio, proximo paso crear una estructura de tipo drive_controller_data_t dentro de statusRobot
            statusRobot.batVoltage = newMotorDataReceived.batVoltage;
            statusRobot.speedMeasRpmR = newMotorDataReceived.speedMeasRpmR;
            statusRobot.speedMeasRpmL = newMotorDataReceived.speedMeasRpmL;
            statusRobot.currentR = newMotorDataReceived.currentR;
            statusRobot.currentL = newMotorDataReceived.currentL;
            statusRobot.posInMetersR = newMotorDataReceived.posInMetersR;
            statusRobot.posInMetersL = newMotorDataReceived.posInMetersR;
            statusRobot.isCharging = newMotorDataReceived.isCharging;
            statusRobot.tempMcb = newMotorDataReceived.boardTemp;
            driveControllerStatusHandler(newMotorDataReceived.statusCode);

            float actual = (((statusRobot.posInMetersL + statusRobot.posInMetersR) / 2) - attitudeControlStat.offsetDistInCms);
            statusRobot.actualDistInCms = actual * 100.00;

            uint16_t maxSpeedLimit = 550;
            #ifdef HARDWARE_PROTOTYPE
                maxSpeedLimit = 2999;
            #elifndef MCB_TORQUE_MODE
                maxSpeedLimit = 999;
            #endif
            
            if (abs(statusRobot.speedMeasRpmL) > maxSpeedLimit || abs(statusRobot.speedMeasRpmR) > maxSpeedLimit) {
                attitudeControlStat.contSafetyMaxSpeed++;
                if (attitudeControlStat.contSafetyMaxSpeed > MAX_CYCLES_LIMIT_SPEED) {
                    setStatusRobot(STATUS_ROBOT_ERROR_LIMIT_SPEED);
                    vTaskDelay(30);
                    setStatusRobot(STATUS_ROBOT_ARMED);
                }
            }
            else {
                attitudeControlStat.contSafetyMaxSpeed = 0;
            }
        }

        contMcbTimeout++;
        if(contMcbTimeout > driveControllerTicksTimeout) {
            statusRobot.isMcbConnected = false;
            setStatusRobot(STATUS_ROBOT_ERROR_MCB_CONNECTION);
        }

        xQueueSend(driveControllerMotorQueue, &speedMotors, 0);        // Cada 5ms aprox
    }
}

static void attitudeControl(void *pvParameters){
    float targetLinearRpm = 0.00;       // velocidad lineal en RPM * 10
    uint8_t isYawControlEnabled = false;
    const char *TAG = "AttitudeControlTask";
    uint8_t cont = 0;

    TickType_t lastWake = xTaskGetTickCount();

    while(true) {
        if (statusRobot.statusCode == STATUS_ROBOT_STABILIZED || statusRobot.statusCode == STATUS_ROBOT_TEST_MODE) {

            if (!statusRobot.dirControl.angularVel) {     // Yaw control

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
                // Yaw manual control: Convierto la velocidad angular rad/s a velocidad de los motores para rotar a esa velocidad
                float wheelLinearVelocity = (statusRobot.dirControl.angularVel / 100.00) * (WHEEL_BASE/2.00);
                attitudeControlMotor.motorR = mps2rpm(wheelLinearVelocity);
                attitudeControlMotor.motorL = attitudeControlMotor.motorR * -1;
            }

            if (!statusRobot.dirControl.linearVel) {     // Pos control
                if (attitudeControlStat.attMode != ATT_MODE_POS_CONTROL) {
                    attitudeControlStat.setPointPosCms = statusRobot.actualDistInCms;           // TODO: podria setearlo cuando llegue a 0 la velocidad de los motores
                    statusRobot.localConfig.pids[PID_POS].setPoint = statusRobot.actualDistInCms;
                    pidSetSetPoint(PID_POS, statusRobot.actualDistInCms);
                    pidSetEnable(PID_POS);
                    attitudeControlStat.attMode = ATT_MODE_POS_CONTROL;
                    ESP_LOGI(TAG,"Enable POS_CONTROL");
                }
         
                targetLinearRpm = (pidCalculate(PID_POS, statusRobot.actualDistInCms) * mps2rpm(MAX_VELOCITY_CONTROL_IN_MPS)) / 10.00; 

                if (cont++ > 10) {
                    cont = 0;
                    ESP_LOGI("posControl", "targetLinearRpm: %f, actualDist: %f", targetLinearRpm, statusRobot.actualDistInCms);
                }
            }
            else {
                float linearVelMps = statusRobot.dirControl.linearVel / 100.00;
                linearVelMps = fmaxf(-MAX_VELOCITY_MPS_CONTROL, fminf(MAX_VELOCITY_MPS_CONTROL, linearVelMps)); // Limito la velocidad maxima permitida

                // targetLinearVel = mps2rpm(statusRobot.dirControl.linearVel / 10.00);     // TODO: es linearVel /100(normalizo) * 10 (para que de en m/s *10)
                targetLinearRpm = mps2rpm(linearVelMps) / 10.00; 
                if (attitudeControlStat.attMode != ATT_MODE_MANUAL_CONTROL) {
                    pidSetDisable(PID_POS);
                    statusRobot.localConfig.pids[PID_POS].setPoint = 0.00;
                    attitudeControlStat.attMode = ATT_MODE_MANUAL_CONTROL;
                    ESP_LOGI(TAG,"Enable MANUAL_CONTROL");
                }
            }

            statusRobot.localConfig.pids[PID_SPEED].setPoint = targetLinearRpm;
            pidSetSetPoint(PID_SPEED, targetLinearRpm);

            int16_t meanSpeedMeas = (statusRobot.speedMeasRpmR - statusRobot.speedMeasRpmL) / 20.00;       // velocidad maxima deberia ser 1000
            float desiredAngleControl = (float)(pidCalculate(PID_SPEED, meanSpeedMeas) * MAX_ANGLE_CONTROL); 

            if (statusRobot.statusCode != STATUS_ROBOT_TEST_MODE) {           // TODO: buscar un mejor mecanismo
                statusRobot.localConfig.pids[PID_ANGLE].setPoint = desiredAngleControl; 
            }
        }
        else {
            if (isYawControlEnabled) {
                isYawControlEnabled = false;
            }
        }

        // ESP_LOGI("imuControlTask", "posInMetersR: %f", statusRobot.posInMetersR);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_PID_SECONDARY_MS));
    }
}

static void commsManager(void *pvParameters) {
    const char *TAG = "commsManager";
    uint8_t socketClientsConnected = 0, lastSocketClientsConnected = 0;
    bool networkState = false, lastNetworkState = false;
    pid_settings_comms_t    newPidSettings;
    command_app_raw_t       newCommand;
    velocity_command_t      newControl;

    while(true) {
        if (xQueueReceive(receiveControlQueueHandler,&newControl,0)) {
            statusRobot.dirControl.linearVel = newControl.linear_vel;
            statusRobot.dirControl.angularVel = newControl.angular_vel;
        }
        
        if (xQueueReceive(newPidParamsQueueHandler,&newPidSettings,0)) {
            pidSetConstants(newPidSettings.indexPid, newPidSettings.kp, newPidSettings.ki, newPidSettings.kd);
            if (newPidSettings.indexPid == PID_ANGLE) {
                pidSetSetPoint(PID_ANGLE, 0);
                statusRobot.localConfig.pids[newPidSettings.indexPid].setPoint = 0;      
            }           
            statusRobot.localConfig.pids[newPidSettings.indexPid].kp = newPidSettings.kp;
            statusRobot.localConfig.pids[newPidSettings.indexPid].ki = newPidSettings.ki;
            statusRobot.localConfig.pids[newPidSettings.indexPid].kd = newPidSettings.kd;
                 
            ESP_LOGI(TAG, "\n**\tNuevos parametros %d:\t**\n*\tP: %.3f\t\t*\n*\tI: %.3f\t\t*\n*\tD: %.3f\t\t*\n*\tsafety limits: %.2f\t*\n",newPidSettings.indexPid,newPidSettings.kp,newPidSettings.ki,newPidSettings.kd,newPidSettings.safetyLimits);              
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

                case COMMAND_PID_ANGLE_TEST:
                    if(statusRobot.statusCode == STATUS_ROBOT_STABILIZED) {
                        // const char *msg = (newCommand.value == 0) ? "Izq" : "Der";
                        ESP_LOGI(TAG,"Init test PID angle");
                        xTaskCreate(PidAngleStepResponseTask, "pid angle test", 4096, NULL, configMAX_PRIORITIES - 3, NULL);
                    } else {
                        ESP_LOGI(TAG,"Error status is not stabilized");
                    }
                break;

                case COMMAND_SAVE_LOCAL_CONFIG:
                    ESP_LOGI(TAG,"Guardando parametros...");
                    storageLocalConfig(statusRobot.localConfig);
                    sendLocalConfig(statusRobot.localConfig);
                break;

                case COMMAND_GET_LOCAL_CONFIG:
                    ESP_LOGI(TAG,"Enviando parametros...");
                    sendLocalConfig(statusRobot.localConfig);
                break;

                case COMMAND_MOVE_DISTANCE:
                    ESP_LOGI(TAG,"Move distance command, distance: %f",newCommand.value / PRECISION_DECIMALS_COMMS);
                    attitudeControlStat.setPointPosCms += newCommand.value;
                    statusRobot.localConfig.pids[PID_POS].setPoint = attitudeControlStat.setPointPosCms;
                    pidSetSetPoint(PID_POS,attitudeControlStat.setPointPosCms);
                break;

                case COMMAND_MOVE_ABS_YAW:
                    float yawAngle = (uint16_t)newCommand.value / PRECISION_DECIMALS_COMMS;
                    ESP_LOGI(TAG,"Move absolute angle: %f",yawAngle);
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

        if (xQueueReceive(collisionSensorsQueue, statusRobot.collisionSensors, 0)) {
            // ESP_LOGI(TAG, " distance: FR: %.02f cm\tFL: %.02f cm\tRR: %.02f cm\tRL: %.02f cm", statusRobot.collisionSensors[ULTRASONIC_FRONT_RIGHT], statusRobot.collisionSensors[ULTRASONIC_FRONT_LEFT], statusRobot.collisionSensors[ULTRASONIC_REAR_RIGHT], statusRobot.collisionSensors[ULTRASONIC_REAR_LEFT]);  
        }

        xQueuePeek(networkStateQueueHandler, &networkState, 0);
        if (networkState != lastNetworkState) {
            ESP_LOGI(TAG, "Nuevo estado de conexion wifi: %d", networkState);
            lastNetworkState = networkState;
        }

        xQueuePeek(socketConnectionStateQueueHandler, &socketClientsConnected,0);       // Leo el ultimo valor emitido, sin sacarlo de la queue
        if (socketClientsConnected > 0 && (networkState || NAV_CONNECTION_SERIAL )) {
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
                .speedMeasMsR = rpm2mps(statusRobot.speedMeasRpmR) * PRECISION_DECIMALS_COMMS,
                .speedMeasMsL = rpm2mps(statusRobot.speedMeasRpmL) * PRECISION_DECIMALS_COMMS,
                .posWheelR = statusRobot.posInMetersR * PRECISION_DECIMALS_COMMS,
                .posWheelL = statusRobot.posInMetersL * PRECISION_DECIMALS_COMMS,
                .currentR = statusRobot.currentR,                               // Ya esta multiplicada por 100 desde la MCB
                .currentL = statusRobot.currentL,                               // Ya esta multiplicada por 100 desde la MCB
                .pitch =  statusRobot.actualPitch * PRECISION_DECIMALS_COMMS,
                .roll = statusRobot.actualRoll * PRECISION_DECIMALS_COMMS,
                .yaw = statusRobot.actualYaw * PRECISION_DECIMALS_COMMS,
                .collisionSensors = { 
                    statusRobot.collisionSensors[ULTRASONIC_FRONT_LEFT] * PRECISION_DECIMALS_COMMS,
                    statusRobot.collisionSensors[ULTRASONIC_FRONT_RIGHT] * PRECISION_DECIMALS_COMMS,
                    statusRobot.collisionSensors[ULTRASONIC_REAR_LEFT] * PRECISION_DECIMALS_COMMS,
                    statusRobot.collisionSensors[ULTRASONIC_REAR_RIGHT] * PRECISION_DECIMALS_COMMS
                },
                .posInMeters = statusRobot.actualDistInCms,
                .outputYawControl = statusRobot.outputYawControl * PRECISION_DECIMALS_COMMS,
                .setPointAngle = statusRobot.localConfig.pids[PID_ANGLE].setPoint * PRECISION_DECIMALS_COMMS,
                .setPointPos = statusRobot.localConfig.pids[PID_POS].setPoint,                                          // No lo multiplico, para mandarlo en mts
                .setPointYaw = statusRobot.localConfig.pids[PID_YAW].setPoint * PRECISION_DECIMALS_COMMS,
                .setPointSpeed = rpm2mps(statusRobot.localConfig.pids[PID_SPEED].setPoint) * PRECISION_DECIMALS_COMMS * -10,      // El setpoint en el PID es RPM/10
                .statusCode = statusRobot.statusCode
            };
            sendDynamicData(newData);
        }

        vTaskDelay(pdMS_TO_TICKS(25));
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
            statusRobot.speedTargetL = testMotor;
            speedMotors.motorL = testMotor;
        } else {
            statusRobot.speedTargetR = testMotor;
            speedMotors.motorR = testMotor;
        }
        vTaskDelay(25);
    }

    vTaskDelay(pdMS_TO_TICKS(TIME_CLEAN_WHEELS_MS));
    statusRobot.speedTargetL = 0;
    statusRobot.speedTargetR = 0;
    speedMotors.motorL = 0;
    speedMotors.motorR = 0;
    speedMotors.enable = false;
    vTaskDelay(pdMS_TO_TICKS(TIME_CLEAN_WHEELS_MS));

    vTaskResume(imuTaskHandler);
    setStatusRobot(STATUS_ROBOT_ARMED);
    vTaskDelete(NULL);
}

static void PidAngleStepResponseTask(void *pvParameters) {
    const uint16_t delayStep = 500;
    const float stepAmpAngle = 10.0;

    setStatusRobot(STATUS_ROBOT_TEST_MODE);

    pidSetDisable(PID_SPEED);
    pidSetDisable(PID_POS);
    statusRobot.localConfig.pids[PID_SPEED].setPoint = 0.0;
    statusRobot.localConfig.pids[PID_POS].setPoint = 0.0;
    pidSetSetPoint(PID_SPEED, 0.0);
    pidSetSetPoint(PID_POS, 0.0);
    
    statusRobot.localConfig.pids[PID_ANGLE].setPoint = 0;
    vTaskDelay(pdMS_TO_TICKS(delayStep));
    statusRobot.localConfig.pids[PID_ANGLE].setPoint = -stepAmpAngle;
    vTaskDelay(pdMS_TO_TICKS(delayStep));
    statusRobot.localConfig.pids[PID_ANGLE].setPoint = stepAmpAngle;
    vTaskDelay(pdMS_TO_TICKS(delayStep));
    statusRobot.localConfig.pids[PID_ANGLE].setPoint = 0;
    vTaskDelay(pdMS_TO_TICKS(delayStep));

    pidClearTerms(PID_POS);
    pidClearTerms(PID_SPEED);
    pidSetEnable(PID_POS);
    pidSetEnable(PID_SPEED);
    setStatusRobot(STATUS_ROBOT_STABILIZED);
    vTaskDelete(NULL);
}

void app_main() {
    const char *TAG = "app_main";
    // gpio_set_direction(PIN_LED , GPIO_MODE_OUTPUT);
    // gpio_set_level(PIN_LED, 1);

    statusRobot.localConfig.versionFirmware = VERSION_FIRMWARE;

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_OSCILO], PIN_FUNC_GPIO);
    gpio_set_direction(PIN_OSCILO , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_OSCILO, 1);

    #ifdef HARDWARE_MAINBOARD
        gpio_set_direction(GPIO_INPUT_NAV_COMMS_MODE , GPIO_MODE_INPUT);
        gpio_set_pull_mode(GPIO_INPUT_NAV_COMMS_MODE, GPIO_PULLUP_ONLY);
    #endif

    receiveControlQueueHandler = xQueueCreate(1, sizeof(velocity_command_t));
    newPidParamsQueueHandler = xQueueCreate(1, sizeof(pid_settings_comms_t));
    newCommandQueueHandler = xQueueCreate(1,  sizeof(command_app_raw_t));
    driveControllerMotorQueue = xQueueCreate(1, sizeof(drive_controller_motor_control_t));
    driveControllerDataReceiveQueue = xQueueCreate(1, sizeof(drive_controller_data_t));
    mpu6050QueueHandler = xQueueCreate(1, sizeof(vector_queue_t));
    socketConnectionStateQueueHandler = xQueueCreate(1, sizeof(uint8_t));
    networkStateQueueHandler = xQueueCreate(1, sizeof(bool));
    collisionSensorsQueue = xQueueCreate(1, sizeof(float)*4);

    xStreamBufferSender = xStreamBufferCreate(STREAM_BUFFER_SIZE, STREAM_BUFFER_LENGTH_TRIGGER);
    xStreamBufferReceiver = xStreamBufferCreate(STREAM_BUFFER_SIZE, STREAM_BUFFER_LENGTH_TRIGGER);
    
    #ifdef HARDWARE_MAINBOARD
        // newMcbQueueHandler = xQueueCreate(1,sizeof(rx_motor_control_board_t));       // TODO: eliminar
        xTaskCreate(statusLedHandler, "status led handler", 2048, socketConnectionStateQueueHandler, 2, NULL);
    #endif

    setStatusRobot(STATUS_ROBOT_INIT);

    #ifdef HARDWARE_MAINBOARD
        // TORQUE MODE:
        #ifdef MCB_TORQUE_MODE
            statusRobot.localConfig.pids[PID_ANGLE].kp = 0.6;
            statusRobot.localConfig.pids[PID_ANGLE].ki = 0.20;
            statusRobot.localConfig.pids[PID_ANGLE].kd = 0.11;

            statusRobot.localConfig.pids[PID_SPEED].kp = 7.0;
            statusRobot.localConfig.pids[PID_SPEED].ki = 0.05;
            statusRobot.localConfig.pids[PID_SPEED].kd = 0.4;

            statusRobot.localConfig.pids[PID_POS].kp = 1.1;
            statusRobot.localConfig.pids[PID_POS].ki = 0.05;
            statusRobot.localConfig.pids[PID_POS].kd = 0.36;

            statusRobot.localConfig.pids[PID_YAW].kp = 2.50;
            statusRobot.localConfig.pids[PID_YAW].ki = 0.5;
            statusRobot.localConfig.pids[PID_YAW].kd = 2.00;
        #else 
            // SPEED_MODE:
            statusRobot.localConfig.pids[PID_ANGLE].kp = 1.0;
            statusRobot.localConfig.pids[PID_ANGLE].ki = 0.14;
            statusRobot.localConfig.pids[PID_ANGLE].kd = 1.4;

            statusRobot.localConfig.pids[PID_SPEED].kp = 5.86;
            statusRobot.localConfig.pids[PID_SPEED].ki = 1.28;
            statusRobot.localConfig.pids[PID_SPEED].kd = 0.07;

            statusRobot.localConfig.pids[PID_POS].kp = 0.36;
            statusRobot.localConfig.pids[PID_POS].ki = 0.02;
            statusRobot.localConfig.pids[PID_POS].kd = 0.05;

            statusRobot.localConfig.pids[PID_YAW].kp = 1.5;
            statusRobot.localConfig.pids[PID_YAW].ki = 0.5;
            statusRobot.localConfig.pids[PID_YAW].kd = 1.5;
        #endif

        statusRobot.localConfig.safetyLimits = 45;
    #elif defined(HARDWARE_PROTOTYPE)
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

        statusRobot.localConfig.safetyLimits = 45; // 35;
    #endif

    statusRobot.localConfig.pids[PID_ANGLE].setPoint = 0;
    statusRobot.localConfig.pids[PID_SPEED].setPoint = 0;

    ESP_LOGI(TAG, "\n------------------- local config -------------------"); 
    ESP_LOGI(TAG, "safetyLimits: %.02f",statusRobot.localConfig.safetyLimits);
    
    for (uint8_t i=0;i<CANT_PIDS;i++) {
        ESP_LOGI(TAG,"PID %d Params: kp: %.02f\tki: %.02f\tkd: %.02f\tsetPoint: %.02f",i,statusRobot.localConfig.pids[i].kp,statusRobot.localConfig.pids[i].ki,statusRobot.localConfig.pids[i].kd,statusRobot.localConfig.pids[i].setPoint);
    }
    ESP_LOGI(TAG, "\n------------------- local config -------------------\n"); 

    mpu6050_init_t configMpu = {
        .intGpio = GPIO_MPU_INT,
        .sclGpio = GPIO_MPU_SCL,
        .sdaGpio = GPIO_MPU_SDA,
        .priorityTask = MPU_HANDLER_PRIORITY,
        .core = IMU_HANDLER_CORE
    };
    mpu6050_initialize(&configMpu);

    pid_init_t pidConfig;
    pidConfig.pids[PID_ANGLE] = convertPidFloatToStruct(statusRobot.localConfig.pids[PID_ANGLE] ,PERIOD_PID_PRIMARY_MS);
    pidConfig.pids[PID_SPEED] = convertPidFloatToStruct(statusRobot.localConfig.pids[PID_SPEED] ,PERIOD_PID_SECONDARY_MS);
    pidConfig.pids[PID_POS] = convertPidFloatToStruct(statusRobot.localConfig.pids[PID_POS] ,PERIOD_PID_SECONDARY_MS);
    pidConfig.pids[PID_YAW] = convertPidFloatToStruct(statusRobot.localConfig.pids[PID_YAW] ,PERIOD_PID_SECONDARY_MS);
    pidInit(pidConfig);

    #ifdef HARDWARE_MAINBOARD
        ultrasonic_config_t UltrasonicConfig = {
            .gpioTrig = GPIO_ULTRASONIC_TRIG,
            .gpioSensor[ULTRASONIC_FRONT_LEFT] = GPIO_ULTRASONIC_FRONT_L,
            .gpioSensor[ULTRASONIC_FRONT_RIGHT] = GPIO_ULTRASONIC_FRONT_R,
            .gpioSensor[ULTRASONIC_REAR_LEFT] = GPIO_ULTRASONIC_REAR_L,
            .gpioSensor[ULTRASONIC_REAR_RIGHT] = GPIO_ULTRASONIC_REAR_R,
            .updateQueue = collisionSensorsQueue,
        };
        ultrasonicInit(&UltrasonicConfig);
    #endif

    driveControllerInit(driveControllerMotorQueue, driveControllerDataReceiveQueue);

    setStatusRobot(STATUS_ROBOT_ARMED);
    xTaskCreatePinnedToCore(imuControlHandler,"Imu Control",4096,NULL,IMU_HANDLER_PRIORITY,&imuTaskHandler,IMU_HANDLER_CORE);
    xTaskCreatePinnedToCore(attitudeControl,"attitude control",4096,NULL,ATTITUDE_HANDLER_PRIORITY, NULL,IMU_HANDLER_CORE);
    xTaskCreatePinnedToCore(commsManager,"communication manager",4096,NULL,COMM_HANDLER_PRIORITY,NULL,IMU_HANDLER_CORE);

    // ESP_LOGI(TAG, "Wifi mode AP");
    // initWifi(ESP_WIFI_SSID_AP, ESP_WIFI_PASS_AP, WIFI_MODE_AP, networkStateQueueHandler);

    ESP_LOGI(TAG, "Wifi mode STA");
    initWifi(ESP_WIFI_SSID_STA, ESP_WIFI_PASS_STA, WIFI_MODE_STA, networkStateQueueHandler);

    #ifdef HARDWARE_MAINBOARD
        bool isNavSerialEnabled = gpio_get_level(GPIO_INPUT_NAV_COMMS_MODE);
        if (isNavSerialEnabled) {   // Conexion de navegacion via puerto serie
            config_init_nav_t configSerialClient = {
                .numUart = UART_PORT_NAV,
                .txPin = GPIO_NAV_TX,
                .rxPin = GPIO_NAV_RX,
                .baudrate = UART_NAV_BAUD,
                .xStreamBufferSend = xStreamBufferSender,
                .xStreamBufferRecv = xStreamBufferReceiver,
                .connectionQueueHandler = socketConnectionStateQueueHandler,
                .core = COMMS_HANDLER_CORE,
            };
            navComms(&configSerialClient);

            comms_start_up();
        } else {                                            // Conexion de navegacion via SOCKET TCP
            tcp_socket_config_t configSocket = {
                .connectionQueueHandler = socketConnectionStateQueueHandler,
                .xStreamBufferSend = xStreamBufferSender,
                .xStreamBufferRecv = xStreamBufferReceiver
            };
            initTcpServerSocket(configSocket);
            // initTcpClientSocket(configSocket);
        }
    #else 
        tcp_socket_config_t configSocket = {
            .connectionQueueHandler = socketConnectionStateQueueHandler,
            .xStreamBufferSend = xStreamBufferSender,
            .xStreamBufferRecv = xStreamBufferReceiver
        };
        initTcpServerSocket(configSocket);
    #endif

    udpLoggerInit(514); // Inicio modulo de logs

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
#include "driver/gpio.h"
#include "soc/gpio_periph.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "stdio.h"
#include "math.h"
#include "esp_log.h"

#include "main.h"
#include "comms.h"
#include "PID.h"
#include "storage_flash.h"
#include "stepper.h"
#include "mpu6050_wrapper.h"
/* Incluyo componentes */
#include "../components/BT_BLE/include/BT_BLE.h"
#include "../components/CAN_COMMS/include/CAN_MCB.h"
#include "../components/SERVO_CONTROL/include/SERVO_CONTROL.h"

#define GRAPH_ARDUINO_PLOTTER   false
#define MAX_VELOCITY            1000.00


#ifdef HARDWARE_HOVERROBOT
    #define MAX_ANGLE_JOYSTICK          4.0
    #define MAX_ROTATION_RATE_CONTROL   50
#else
    #define MAX_ANGLE_JOYSTICK      8.0
    #define MAX_ROTATION_RATE_CONTROL         100
#endif

extern QueueSetHandle_t newAnglesQueue;                 // Recibo nuevos angulos obtenidos del MPU
QueueSetHandle_t queueMotorControl;                     // Envio nuevos valores de salida para el control de motores
QueueHandle_t queueNewPidParams;                        // Recibo nuevos parametros relacionados al pid
QueueHandle_t queueNewCommand;
QueueHandle_t queueReceiveControl;

status_robot_t statusRobot;                            // Estructura que contiene todos los parametros de status a enviar a la app
output_motors_t speedMotors;
output_motors_t attitudeControlMotor;

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

static void imuControlHandler(void *pvParameters) {
    vector_queue_t newAngles;
    float safetyLimitProm[5];
    uint8_t safetyLimitPromIndex = 0;

    statusRobot.statusCode = STATUS_ROBOT_ARMED;

    while(1) {
        if(xQueueReceive(newAnglesQueue,&newAngles,pdMS_TO_TICKS(10))) {
        
            statusRobot.roll = newAngles.roll;
            statusRobot.pitch = newAngles.pitch;
            statusRobot.yaw = newAngles.yaw;
            statusRobot.tempImu = (uint16_t)newAngles.temp * 10;

            #ifdef HARDWARE_HOVERROBOT
                float angleReference = newAngles.roll * -1;
            #else
                float angleReference = newAngles.pitch;
            #endif
            
            int16_t outputPidMotors = (uint16_t)(pidCalculate(angleReference) * MAX_VELOCITY); 

            // // kickstart for brushless motor
            // #ifdef HARDWARE_HOVERROBOT
            // const uint8_t kickstart = 90;
            // const uint8_t deadband = 90;
            //     if(outputPidMotors < deadband && outputPidMotors >= 0) {
            //         outputPidMotors += kickstart;
            //     }
            //     else if (outputPidMotors >-deadband && outputPidMotors < 0) {
            //         outputPidMotors -= kickstart;
            //     }
            // #endif

            // uint16_t deadBand = 3;
            // if(outputPidMotors > deadBand || outputPidMotors < -deadBand) {
                speedMotors.motorL = outputPidMotors + attitudeControlMotor.motorL;
                speedMotors.motorR = outputPidMotors + attitudeControlMotor.motorR;
            // }
            // else {
            //     speedMotors.motorL = 0;
            //     speedMotors.motorR = 0;
            // }

            speedMotors.motorL = cutSpeedRange(speedMotors.motorL);
            speedMotors.motorR = cutSpeedRange(speedMotors.motorR);

            safetyLimitProm[safetyLimitPromIndex++] = angleReference;
            if (safetyLimitPromIndex>2) {
                safetyLimitPromIndex = 0;
            }
            float angleSafetyLimit = (safetyLimitProm[0] + safetyLimitProm[1] + safetyLimitProm[2]) / 3;

            if (pidGetEnable()) { 
                if (((angleSafetyLimit < (statusRobot.pid.centerAngle-statusRobot.pid.safetyLimits)) || (angleSafetyLimit > (statusRobot.pid.centerAngle+statusRobot.pid.safetyLimits)))) { 
                    pidSetDisable();
                    speedMotors.enable = false;
                    speedMotors.motorL = 0;
                    speedMotors.motorR = 0;
                    statusRobot.statusCode = STATUS_ROBOT_ARMED;
                    printf("DISABLED, SAFETYLIMITS: %f\n",statusRobot.pid.safetyLimits);
                }
            }
            else { 
                if (((angleReference > (statusRobot.pid.centerAngle-1)) && (angleReference < (statusRobot.pid.centerAngle+1)))) { 
                    pidSetEnable();   
                    speedMotors.enable = true;
                    statusRobot.statusCode = STATUS_ROBOT_STABILIZED;                                              
                }
            }
            
            #ifdef HARDWARE_HOVERROBOT
                const int8_t calibratePwmOffset = 3;
                uint16_t velMotorToServoR = (speedMotors.motorR/2) + 500 + calibratePwmOffset;
                uint16_t velMotorToServoL = (speedMotors.motorL/2) + 500 + calibratePwmOffset;
                pwmSetOutput(2,velMotorToServoL);
                pwmSetOutput(3,velMotorToServoR);
                // printf("PWM OUTPUTS: L: %d R: %d\n",velMotorToServoR,velMotorToServoL);
            #else
                xQueueSend(queueMotorControl,&speedMotors,pdMS_TO_TICKS(1));
            #endif
            statusRobot.speedL = speedMotors.motorL;
            statusRobot.speedR = speedMotors.motorR;
            // gpio_set_level(PIN_OSCILO, 0);

            // printf("Pitch: %f\tRoll: %f\tEnablePid: %d\tEnableMotor:%d\n",newAngles.pitch,newAngles.roll,pidGetEnable(),speedMotors.enable);
            
            // ESP_LOGE("IMU_CONTROL_HANDLER", "Pitch: %f\tRoll: %f\tEnablePid: %d\tEnableMotor:%d\n",newAngles.pitch,newAngles.roll,pidGetEnable(),speedMotors.enable);
        }
    }
}

static void attitudeControl(void *pvParameters){

    control_app_t newControlVal;
    output_motors_t newVel; // TODO: para prueba PWM
    uint8_t contpwm = 0;

    while(true) {
        if( xQueueReceive(queueReceiveControl,
                         &newControlVal,
                         ( TickType_t ) 1 ) == pdPASS) {

            // Prueba directa control de motores:
            // attitudeControlMotor.motorL = newControlVal.axis_x * -1 * MAX_ROTATION_RATE_CONTROL + newControlVal.axis_y * -1 * MAX_ROTATION_RATE_CONTROL;
            // attitudeControlMotor.motorR = newControlVal.axis_x *  MAX_ROTATION_RATE_CONTROL + newControlVal.axis_y * -1 * MAX_ROTATION_RATE_CONTROL;
            // setVelMotors(attitudeControlMotor.motorL,attitudeControlMotor.motorR);

            attitudeControlMotor.motorL = (newControlVal.axis_x / 100.00) * -1 * MAX_ROTATION_RATE_CONTROL;
            attitudeControlMotor.motorR = (newControlVal.axis_x / 100.00) *  MAX_ROTATION_RATE_CONTROL;

            float setPoint = statusRobot.pid.centerAngle + (newControlVal.axis_y / 100.00) * MAX_ANGLE_JOYSTICK; // Max 5 grados

            statusRobot.setPoint = setPoint;
            pidSetSetPoint(setPoint);
        } 

        // if (xQueueReceive(queueMotorControl,&newVel,pdMS_TO_TICKS(1))) {


        //     // contpwm++;
        //     // if (contpwm> 99) { 
        //         // contpwm = 60;
        //     // }
        //     uint8_t velMotorToServoR = (newVel.motorR/20) + 50;
        //     uint8_t velMotorToServoL = (newVel.motorL/20) + 50;
        //     pwmSetOutput(2,velMotorToServoL);
        //     pwmSetOutput(3,velMotorToServoR);
        // }
        vTaskDelay(50);
    }
}


static void commsManager(void *pvParameters){
    uint8_t cont1 = 0;
    pid_params_t newPidParams;
    command_app_t newCommand;

    while (1) {
        
        if(xQueueReceive(queueNewPidParams,&newPidParams,0)) {
            pidSetConstants(newPidParams.kp,newPidParams.ki,newPidParams.kd);
            pidSetSetPoint(newPidParams.centerAngle);
            storageWritePidParams(newPidParams);            

            statusRobot.pid = newPidParams;
            printf("\nNuevos parametros:\n\tP: %f\n\tI: %f\n\tD: %f,\n\tcenter: %f\n\tsafety limits: %f\n\n",newPidParams.kp,newPidParams.ki,newPidParams.kd,newPidParams.centerAngle,newPidParams.safetyLimits);              
        }
        if(xQueueReceive(queueNewCommand,&newCommand,0)) {

            switch (newCommand.command) {
                case COMMAND_CALIBRATE_IMU:
                
                    printf("\nreseteado DMP, calibrando ->\n");
                    mpu6050_recalibrate();
                break;
            }            
        }
        if (isBtConnected()) {
            cont1++;
            if(cont1>50){
                cont1 =0;
            }
            statusRobot.batVoltage = 10;
            statusRobot.batPercent = 55;
            statusRobot.batTemp = cont1;
            statusRobot.tempEscs = cont1;
            // uint16_t ordenCode;
            // uint16_t statusCode;

            // TODO: eliminar conversion
            robot_dynamic_data_t newData = {
                .speedR = statusRobot.speedR,
                .speedL = statusRobot.speedL,
                .pitch = statusRobot.pitch * PRECISION_DECIMALS_COMMS,
                .roll = statusRobot.roll * PRECISION_DECIMALS_COMMS,
                .yaw = statusRobot.yaw * PRECISION_DECIMALS_COMMS,
                .setPoint = statusRobot.setPoint * PRECISION_DECIMALS_COMMS,
                .centerAngle = statusRobot.pid.centerAngle * PRECISION_DECIMALS_COMMS,
                .statusCode = statusRobot.statusCode
            };
            sendDynamicData(newData);
        }
        // if(GRAPH_ARDUINO_PLOTTER){
            //Envio log para graficar en arduino serial plotter
            //printf("angle_x:%f,set_point: %f,output_pid: %f, output_motor:%d\n",newAngles[AXIS_ANGLE_Y],CENTER_ANGLE_MOUNTED,outputPid,outputMotors.motorL);
        // }

        // testHardwareVibration();
        // printf(">angle:%f\n>outputMotor:%f\n",angleReference/10.0,statusRobot.speedL/100.0);
        // printf("angle:%f,set_point: %f,kp: %f,ki: %f,kd: %f,output_motor:%d\n",angleReference,statusRobot.setPoint,statusRobot.pid.kp,statusRobot.pid.ki,statusRobot.pid.kd,statusRobot.speedL);
   
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void testHardwareVibration(void) {
    uint8_t flagInc=true;
    int16_t testMotor=0;

    while(true) {
        printf(">angle:%f\n>outputMotor:%f\n",statusRobot.pitch/10.0,statusRobot.speedL/100.0);

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
        xQueueSend(queueMotorControl,&speedMotors,pdMS_TO_TICKS(1));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void ledHandler(void *pvParameters) {

    uint16_t delay = 1000;
    while(1) {
        if (isBtConnected()) {
            delay = 500;
        }
        else {
            delay = 1000;
        }
        gpio_set_level(PIN_LED,1);
        vTaskDelay(pdMS_TO_TICKS(delay));
        gpio_set_level(PIN_LED,0);
        vTaskDelay(pdMS_TO_TICKS(delay));
        // speedMotors.enable = 0xBB;
        // speedMotors.motorL = 0xCC;
        // speedMotors.motorR = 0xDD;
        // xQueueSend(queueMotorControl,&speedMotors,pdMS_TO_TICKS(1));
        // testHardwareVibration();
    }
}

void app_main() {
    pid_params_t readParams={0};

    gpio_set_direction(PIN_LED , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 1);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_OSCILO], PIN_FUNC_GPIO);
    gpio_set_direction(PIN_OSCILO , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_OSCILO, 0);

    queueReceiveControl = xQueueCreate(1, sizeof(control_app_t));
    queueNewPidParams = xQueueCreate(1,sizeof(pid_params_t));
    queueNewCommand = xQueueCreate(1, sizeof(command_app_t));
    queueMotorControl = xQueueCreate(1,sizeof(output_motors_t));

    storageInit();

    btInit(DEVICE_BT_NAME);
    vTaskDelay(1000);

    mpu6050_init_t configMpu = {
        .intGpio = GPIO_MPU_INT,
        .sclGpio = GPIO_MPU_SCL,
        .sdaGpio = GPIO_MPU_SDA,
        .priorityTask = MPU_HANDLER_PRIORITY
    };
    mpu6050_initialize(&configMpu);
    vTaskDelay(pdMS_TO_TICKS(5000));

    readParams = storageReadPidParams();
    // readParams.kd = 0;
    // readParams.ki = 0;
    // readParams.kp = 0.5;
    statusRobot.pid = readParams;
    statusRobot.setPoint = readParams.centerAngle;
    printf("Hardcode Params: center: %f kp: %f , ki: %f , kd: %f,safetyLimits: %f,centerAngle: %f\n",readParams.centerAngle,readParams.kp,readParams.ki,readParams.kd,readParams.safetyLimits,readParams.centerAngle);
    
    pid_init_t pidConfig = {
        .kp = readParams.kp,
        .ki = readParams.ki,
        .kd = readParams.kd,
        .initSetPoint = readParams.centerAngle,
        .sampleTimeInMs = PERIOD_IMU_MS,
    };
    pidInit(pidConfig);

    #ifdef HARDWARE_HOVERROBOT
        // config_init_mcb_t configMcb = {
        //     .numUart = UART_PORT_CAN,
        //     .txPin = GPIO_CAN_TX,
        //     .rxPin = GPIO_CAN_RX
        // };
        // mcbInit(&configMcb);
        pwm_servo_init_t configServos = {
            .hsPwm1Gpio = -1,
            .hsPwm2Gpio = -1,
            .lsPwm1Gpio = GPIO_PWM_R,
            .lsPwm2Gpio = GPIO_PWM_L,
            .lsPwm3Gpio = -1,
            .lsPwm4Gpio = -1,
        };
        pwmServoInit(configServos);
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

    xTaskCreatePinnedToCore(imuControlHandler,"Imu Control Task",4096,NULL,IMU_HANDLER_PRIORITY,NULL,IMU_HANDLER_CORE);
    xTaskCreate(attitudeControl,"attitude control Task",2048,NULL,4,NULL);
    xTaskCreate(commsManager,"communication manager",4096,NULL,COMM_HANDLER_PRIORITY,NULL);
    xTaskCreate(ledHandler,"Led handler",2048,NULL,2,NULL);
}
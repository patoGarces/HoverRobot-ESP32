#include "driver/gpio.h"
#include "soc/gpio_periph.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "stdio.h"
#include "math.h"
#include "driver/i2c.h"

#include "main.h"
#include "comms.h"
#include "PID.h"
#include "storage_flash.h"
#include "stepper.h"

#include "mpu6050_wrapper.h"
/* Incluyo componentes */
#include "../components/BT_CLASSIC/include/BT_CLASSIC.h"
#include "../components/CAN_COMMS/include/CAN_MCB.h"

#define GRAPH_ARDUINO_PLOTTER   false
#define MAX_VELOCITY            1000.00
#define VEL_MAX_CONTROL         100    
#define MAX_ANGLE_JOYSTICK      4.0

#define DEVICE_BT_NAME          "Balancing robot"

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
    float safetyLimitProm[2];
    uint8_t safetyLimitPromIndex = 0;

    // queueMotorControl = xQueueCreate(1,sizeof(output_motors_t));
    statusRobot.statusCode = STATUS_ROBOT_ARMED;

    while(1){
        if(xQueueReceive(newAnglesQueue,&newAngles,pdMS_TO_TICKS(10))) {
            // printf("Yaw: %f\n",newAngles.yaw);
            statusRobot.roll = newAngles.roll;
            statusRobot.pitch = newAngles.pitch;
            statusRobot.yaw = newAngles.yaw;
            statusRobot.tempImu = (uint16_t)newAngles.temp * 10;

            uint16_t outputPidMotors = (uint16_t)(pidCalculate(statusRobot.pitch) * MAX_VELOCITY); 

            if(outputPidMotors >25 || statusRobot.pitch < -25) {
                speedMotors.motorL = outputPidMotors + attitudeControlMotor.motorL;
                speedMotors.motorR = outputPidMotors + attitudeControlMotor.motorR;
            }
            else {
                speedMotors.motorL = 0;
                speedMotors.motorR = 0;
            }

            speedMotors.motorL = cutSpeedRange(speedMotors.motorL);
            speedMotors.motorR = cutSpeedRange(speedMotors.motorR);

            safetyLimitProm[safetyLimitPromIndex++] = statusRobot.pitch;
            if (safetyLimitPromIndex>1) {
                safetyLimitPromIndex = 0;
            }
            float angleSafetyLimit = (safetyLimitProm[0] + safetyLimitProm[1]) / 2;

            if (pidGetEnable()) { 
                if (((angleSafetyLimit < (statusRobot.pid.centerAngle-statusRobot.pid.safetyLimits)) || (angleSafetyLimit > (statusRobot.pid.centerAngle+statusRobot.pid.safetyLimits)))) { 
                    pidSetDisable();
                    speedMotors.enable = false;
                    speedMotors.motorL = 0;
                    speedMotors.motorR = 0;
                    statusRobot.statusCode = STATUS_ROBOT_ARMED;
                }
            }
            else { 
                if (((statusRobot.pitch > (statusRobot.pid.centerAngle-1)) && (statusRobot.pitch < (statusRobot.pid.centerAngle+1)))) { 
                    pidSetEnable();   
                    speedMotors.enable = true;
                    statusRobot.statusCode = STATUS_ROBOT_STABILIZED;                                                   
                }
            }

            xQueueSend(queueMotorControl,&speedMotors,pdMS_TO_TICKS(1));
            statusRobot.speedL = speedMotors.motorL;
            statusRobot.speedR = speedMotors.motorR;
            // gpio_set_level(PIN_OSCILO, 0);
        }
    }
}

static void attitudeControl(void *pvParameters){

    queueReceiveControl = xQueueCreate(1, sizeof(control_app_t));

    control_app_t newControlVal;

    while(true){
        if( xQueueReceive(queueReceiveControl,
                         &newControlVal,
                         ( TickType_t ) 1 ) == pdPASS ){

            // Prueba directa control de motores:
            // attitudeControlMotor.motorL = newControlVal.axis_x * -1 * VEL_MAX_CONTROL + newControlVal.axis_y * -1 * VEL_MAX_CONTROL;
            // attitudeControlMotor.motorR = newControlVal.axis_x *  VEL_MAX_CONTROL + newControlVal.axis_y * -1 * VEL_MAX_CONTROL;
            // setVelMotors(attitudeControlMotor.motorL,attitudeControlMotor.motorR);

            attitudeControlMotor.motorL = (newControlVal.axis_x / 100.00) * -1 * VEL_MAX_CONTROL;
            attitudeControlMotor.motorR = (newControlVal.axis_x / 100.00) *  VEL_MAX_CONTROL;

            float setPoint = statusRobot.pid.centerAngle + (newControlVal.axis_y / 100.00) * MAX_ANGLE_JOYSTICK; // Max 5 grados

            statusRobot.setPoint = setPoint;
            pidSetSetPoint(setPoint);
        } 
    }
}


static void commsManager(void *pvParameters){
    uint8_t cont1 = 0;
    pid_params_t newPidParams;
    command_app_t newCommand;

    queueNewPidParams = xQueueCreate(1,sizeof(pid_params_t));
    queueNewCommand = xQueueCreate(1, sizeof(command_app_t));

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
        if (btIsConnected()) {
            cont1++;
            if(cont1>50){
                cont1 =0;
            }
            statusRobot.header = HEADER_TX_KEY_STATUS;
            statusRobot.batVoltage = 10;
            statusRobot.batPercent = 55;
            statusRobot.batTemp = cont1;
            statusRobot.tempEscs = cont1;
            // uint16_t ordenCode;
            // uint16_t statusCode;
            sendStatus(statusRobot);
        }
        // if(GRAPH_ARDUINO_PLOTTER){
            //Envio log para graficar en arduino serial plotter
            //printf("angle_x:%f,set_point: %f,output_pid: %f, output_motor:%d\n",newAngles[AXIS_ANGLE_Y],CENTER_ANGLE_MOUNTED,outputPid,outputMotors.motorL);
        // }

        // testHardwareVibration();
        // printf(">angle:%f\n>outputMotor:%f\n",statusRobot.pitch/10.0,statusRobot.speedL/100.0);
        // printf("angle:%f,set_point: %f,kp: %f,ki: %f,kd: %f,output_motor:%d\n",statusRobot.pitch,statusRobot.setPoint,statusRobot.pid.kp,statusRobot.pid.ki,statusRobot.pid.kd,statusRobot.speedL);
   
        vTaskDelay(pdMS_TO_TICKS(50));
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
        speedMotors.motorL = testMotor*5;
        speedMotors.motorR = testMotor*5;
        xQueueSend(queueMotorControl,&speedMotors,pdMS_TO_TICKS(1));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main() {
    pid_params_t readParams={0};

    gpio_set_direction(PIN_LED , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 0);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_OSCILO], PIN_FUNC_GPIO);
    gpio_set_direction(PIN_OSCILO , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_OSCILO, 0);

    storageInit();

    btInit(DEVICE_BT_NAME);

    mpu6050_init_t configMpu = {
        .intGpio = GPIO_MPU_INT,
        .sclGpio = GPIO_MPU_SCL,
        .sdaGpio = GPIO_MPU_SDA,
        .priorityTask = MPU_HANDLER_PRIORITY
    };
    mpu6050_initialize(&configMpu);
    vTaskDelay(pdMS_TO_TICKS(5000));

    readParams = storageReadPidParams();
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
        config_init_mcb_t configMcb = {
            .numUart = UART_PORT_CAN,
            .txPin = GPIO_CAN_TX,
            .rxPin = GPIO_CAN_RX
        };
        mcbInit(&configMcb);
    #endif

    xTaskCreatePinnedToCore(imuControlHandler,"Imu Control Task",4096,NULL,IMU_HANDLER_PRIORITY,NULL,IMU_HANDLER_CORE);
    xTaskCreate(attitudeControl,"attitude control Task",2048,NULL,4,NULL);
    xTaskCreate(commsManager,"communication manager",4096,NULL,3,NULL);

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

    // queueMotorControl = xQueueCreate(1,sizeof(output_motors_t));// TODO: eliminar


    while(1) {
        gpio_set_level(PIN_LED,1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(PIN_LED,0);
        vTaskDelay(pdMS_TO_TICKS(200));
        // speedMotors.enable = 0xBB;
        // speedMotors.motorL = 0xCC;
        // speedMotors.motorR = 0xDD;
        // xQueueSend(queueMotorControl,&speedMotors,pdMS_TO_TICKS(1));
        // testHardwareVibration();
    }
}
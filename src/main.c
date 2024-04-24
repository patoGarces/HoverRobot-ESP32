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

#define GRAPH_ARDUINO_PLOTTER   false
#define MAX_VELOCITY            1000.00
#define VEL_MAX_CONTROL         200    
#define DEVICE_BT_NAME          "Balancing robot"

extern QueueSetHandle_t newAnglesQueue;                 // Recibo nuevos angulos obtenidos del MPU
QueueHandle_t queueNewPidParams;                        // Recibo nuevos parametros relacionados al pid
QueueSetHandle_t outputMotorQueue;                      // Envio nuevos valores de salida para el control de motores

QueueHandle_t queueReceiveControl;

status_robot_t statusToSend;                            // Estructura que contiene todos los parametros de status a enviar a la app

output_motors_t attitudeControlMotor;

static void imuControlHandler(void *pvParameters){
    float newAngles[3];
    output_motors_t outputMotors;
    float safetyLimitProm[2];
    uint8_t safetyLimitPromIndex = 0;

    outputMotorQueue = xQueueCreate(5,sizeof(output_motors_t));

    pidSetEnable();

    while(1){
        if(xQueueReceive(newAnglesQueue,&newAngles,pdMS_TO_TICKS(10))){

            statusToSend.roll = newAngles[AXIS_ANGLE_X];
            statusToSend.pitch = newAngles[AXIS_ANGLE_Y];
            statusToSend.yaw = newAngles[AXIS_ANGLE_Z];

            uint16_t outputPidMotors = (uint16_t)(pidCalculate(statusToSend.pitch) * MAX_VELOCITY); 

            if(outputPidMotors > 50 || statusToSend.pitch < -50) {
                outputMotors.motorL = outputPidMotors + attitudeControlMotor.motorL;
                outputMotors.motorR = outputPidMotors + attitudeControlMotor.motorR;
            }
            else {
                outputMotors.motorL = 0;
                outputMotors.motorR = 0;
            }

            if (outputMotors.motorL > 1000) {
                outputMotors.motorL = 1000;
            }
            if (outputMotors.motorR > 1000) {
                outputMotors.motorR = 1000;
            }
            if (outputMotors.motorL < -1000) {
                outputMotors.motorL = -1000;
            }
            if (outputMotors.motorR < -1000) {
                outputMotors.motorR = -1000;
            }
            setVelMotors(outputMotors.motorL,outputMotors.motorR);

            statusToSend.speedL = outputMotors.motorL;
            statusToSend.speedR = outputMotors.motorR;

            safetyLimitProm[safetyLimitPromIndex++] = newAngles[AXIS_ANGLE_Y];
            if (safetyLimitPromIndex>1) {
                safetyLimitPromIndex = 0;
            }
            float angleSafetyLimit = (safetyLimitProm[0] + safetyLimitProm[1]) / 2;

            if (pidGetEnable()) { 
                if (((angleSafetyLimit < (CENTER_ANGLE_MOUNTED-statusToSend.pid.safetyLimits)) || (angleSafetyLimit > (CENTER_ANGLE_MOUNTED+statusToSend.pid.safetyLimits)))){ 
                    setVelMotors(0,0);
                    pidSetDisable();
                    statusToSend.status_code = STATUS_ROBOT_DISABLE;
                    
                    printf("\n\nSAFETY LIMITS REACHED: angle: %f, safetyLimit: %f\n\n",newAngles[AXIS_ANGLE_Y],statusToSend.pid.safetyLimits); 
                }
            }
            else { 
                if (((newAngles[AXIS_ANGLE_Y] > (CENTER_ANGLE_MOUNTED-1)) && (newAngles[AXIS_ANGLE_Y] < (CENTER_ANGLE_MOUNTED+1)))) { 
                    pidSetEnable();   
                    statusToSend.status_code = STATUS_ROBOT_STABILIZED;                                                   
                }
                else {
                    pidSetDisable();
                }
            }
            // gpio_set_level(PIN_OSCILO, 0);
        }
    }
}

static void updateParams(void *pvParameters){

    pid_params_t newPidParams;

    queueNewPidParams = xQueueCreate(1,sizeof(pid_params_t));

    while (1){
        
        if(xQueueReceive(queueNewPidParams,&newPidParams,0)){
            newPidParams.centerAngle += CENTER_ANGLE_MOUNTED;
            pidSetConstants(newPidParams.kp,newPidParams.ki,newPidParams.kd);
            pidSetSetPoint(newPidParams.centerAngle);
            storageWritePidParams(newPidParams);            

            statusToSend.pid = newPidParams;
            printf("\nNuevos parametros:\n\tP: %f\n\tI: %f\n\tD: %f,\n\tcenter: %f\n\tsafety limits: %f\n\n",newPidParams.kp,newPidParams.ki,newPidParams.kd,newPidParams.centerAngle,newPidParams.safetyLimits);              
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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

            float setPoint = statusToSend.pid.centerAngle + (newControlVal.axis_y / 100.00) * 5.00; // Max 5 grados

            statusToSend.setPoint = setPoint;
            pidSetSetPoint(setPoint);
        } 
    }
}

void testHardwareVibration(){
    uint8_t flagInc=true;
    int16_t testMotor=0;

    // vTaskDelay(pdMS_TO_TICKS(5000));
    while(true) {
        printf(">angle:%f\n>outputMotor:%f\n",statusToSend.pitch/10.0,statusToSend.speedL/100.0);

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
                setVelMotors(0,0);
                // break;
            }
        }
        statusToSend.speedL = testMotor*10;
        statusToSend.speedR = testMotor*10;
        setVelMotors(statusToSend.speedL,statusToSend.speedR);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void app_main() {
    uint8_t cont1=0;
    pid_params_t readParams={0};

    gpio_set_direction(PIN_LED , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 0);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_OSCILO], PIN_FUNC_GPIO);
    gpio_set_direction(PIN_OSCILO , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_OSCILO, 0);

    storageInit();

    btInit(DEVICE_BT_NAME);

    mpu6050_initialize();
    vTaskDelay(pdMS_TO_TICKS(3000));

    readParams = storageReadPidParams();
    // TODO: Eliminar
    readParams.kp = 3.3;
    readParams.ki = 2;
    readParams.kd = 0.2;
    readParams.safetyLimits = 70;
    readParams.centerAngle = 0.0;
    statusToSend.pid = readParams;
    statusToSend.setPoint = readParams.centerAngle;
    printf("Hardcode Params: center: %f kp: %f , ki: %f , kd: %f,safetyLimits: %f,centerAngle: %f\n",readParams.centerAngle,readParams.kp,readParams.ki,readParams.kd,readParams.safetyLimits,readParams.centerAngle);
    
    pid_init_t pidConfig = {
        .kp = readParams.kp,
        .ki = readParams.ki,
        .kd = readParams.kd,
        .initSetPoint = readParams.centerAngle,
        .sampleTimeInMs = PERIOD_IMU_MS,
    };
    pidInit(pidConfig);

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
    enableMotors();

    xTaskCreatePinnedToCore(imuControlHandler,"Imu Control Task",4096,NULL,IMU_HANDLER_PRIORITY,NULL,IMU_HANDLER_CORE);
    xTaskCreate(updateParams,"Update Params Task",2048,NULL,3,NULL);
    xTaskCreate(attitudeControl,"attitude control Task",2048,NULL,4,NULL);

    setVelMotors(0,0);

    while(1) {
        
        if (btIsConnected()) {
            cont1++;
            if(cont1>50){
                cont1 =0;
            }
            statusToSend.header = HEADER_COMMS;
            statusToSend.bat_voltage = 10;
            statusToSend.bat_percent = 55;
            statusToSend.batTemp = 100-cont1;
            statusToSend.temp_uc_control = cont1;
            statusToSend.temp_uc_main = 123-cont1; 
            // statusToSend.status_code = 0;

            sendStatus(statusToSend);
        }
        // gpio_set_level(PIN_LED,1);
        vTaskDelay(pdMS_TO_TICKS(50));
        // gpio_set_level(PIN_LED,0);
        // vTaskDelay(pdMS_TO_TICKS(100));

        // if(GRAPH_ARDUINO_PLOTTER){
            //Envio log para graficar en arduino serial plotter
            //printf("angle_x:%f,set_point: %f,output_pid: %f, output_motor:%d\n",newAngles[AXIS_ANGLE_Y],CENTER_ANGLE_MOUNTED,outputPid,outputMotors.motorL);
        // }

        // testHardwareVibration();

        // printf(">angle:%f\n>outputMotor:%f\n",statusToSend.pitch/10.0,statusToSend.speedL/100.0);

        printf("angle:%f,set_point: %f,kp: %f,ki: %f,kd: %f,output_motor:%d\n",statusToSend.pitch,statusToSend.setPoint,statusToSend.pid.kp,statusToSend.pid.ki,statusToSend.pid.kd,statusToSend.speedL);
    }
}
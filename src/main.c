#include "driver/gpio.h"
#include "soc/gpio_periph.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "stdio.h"

#include "main.h"
#include "comms.h"
#include "PID.h"
#include "storage_flash.h"
#include "stepper.h"

/* Incluyo componentes */
#include "../components/MPU6050/include/MPU6050.h"
#include "../components/BT_CLASSIC/include/BT_CLASSIC.h"

#define GRAPH_ARDUINO_PLOTTER   false
#define MAX_VELOCITY            1000
#define VEL_MAX_CONTROL         5    
#define DEVICE_BT_NAME          "Balancing robot"

extern QueueSetHandle_t newAnglesQueue;                 // Recibo nuevos angulos obtenidos del MPU
QueueHandle_t queueNewPidParams;                        // Recibo nuevos parametros relacionados al pid
QueueSetHandle_t outputMotorQueue;                      // Envio nuevos valores de salida para el control de motores

QueueHandle_t queueReceiveControl;

status_robot_t statusToSend;                            // Estructura que contiene todos los parametros de status a enviar a la app

output_motors_t attitudeControlMotor;

static void imuControlHandler(void *pvParameters){
    float newAngles[3];
    float outputPid;
    output_motors_t outputMotors;

    uint32_t cont = 0;

    outputMotorQueue = xQueueCreate(5,sizeof(output_motors_t));

    while(1){

        if(xQueueReceive(newAnglesQueue,&newAngles,10)){

            // if( getEnablePid() ){
            //     gpio_set_level(PIN_OSCILO, 1);
            // }
            statusToSend.pitch = newAngles[AXIS_ANGLE_Y];
            statusToSend.roll = newAngles[AXIS_ANGLE_X];
            statusToSend.yaw = newAngles[AXIS_ANGLE_Z];

            outputPid = pidCalculate(newAngles[AXIS_ANGLE_Y]) *-1; 
            outputMotors.motorL = outputPid * MAX_VELOCITY;
            outputMotors.motorR = outputMotors.motorL;

            // xQueueSend(outputMotorQueue,(void*) &outputMotors,0);                           // TODO: falta recibir los datos de esta cola y enviarlos a los motores
            setVelMotors(outputMotors.motorL,outputMotors.motorR);

            statusToSend.speedL = outputMotors.motorL;
            statusToSend.speedR = outputMotors.motorR;

            // Envio data a los motores
            // sendMotorData(outputMotors.motorR,outputMotors.motorL,0x00,0x00);            // TODO: controlar el enable

            if(!getEnablePid()){ 
                if((( newAngles[AXIS_ANGLE_Y] > (CENTER_ANGLE_MOUNTED-1)) && ( newAngles[AXIS_ANGLE_Y] < (CENTER_ANGLE_MOUNTED+1) )) ){ 
                    enableMotors();
                    setEnablePid();   
                    statusToSend.status_code = STATUS_ROBOT_STABILIZED;                                                       
                }
                else{
                    disableMotors();
                }
            }
            else{ 
                if((( newAngles[AXIS_ANGLE_Y] < (CENTER_ANGLE_MOUNTED-statusToSend.safetyLimits)) || ( newAngles[AXIS_ANGLE_Y] > (CENTER_ANGLE_MOUNTED+statusToSend.safetyLimits) )) ){ 
                    disableMotors();
                    setDisablePid();
                    statusToSend.status_code = STATUS_ROBOT_DISABLE; 
                }
            }

            if(GRAPH_ARDUINO_PLOTTER){
                //Envio log para graficar en arduino serial plotter
                printf("angle_x:%f,set_point: %f,output_pid: %f, output_motor:%d\n",newAngles[AXIS_ANGLE_Y],CENTER_ANGLE_MOUNTED,outputPid,outputMotors.motorL);
            }
            // gpio_set_level(PIN_OSCILO, 0);

            cont++;
            if(cont>10){
                printf("angle:%f,set_point: %f,output_pid: %f, output_motor:%d\n",newAngles[AXIS_ANGLE_Y],CENTER_ANGLE_MOUNTED,outputPid,outputMotors.motorL);
                cont=0;
            }
        }
    }
}

static void updateParams(void *pvParameters){

    pid_params_t newPidParams;

    queueNewPidParams = xQueueCreate(1,sizeof(pid_params_t));

    while (1){
        
        if(xQueueReceive(queueNewPidParams,&newPidParams,0)){
            newPidParams.center_angle += CENTER_ANGLE_MOUNTED;
            pidSetConstants(newPidParams.kp,newPidParams.ki,newPidParams.kd);
            pidSetPointAngle(newPidParams.center_angle);
            storageWritePidParams(newPidParams);            

            statusToSend.P = newPidParams.kp*100;  
            statusToSend.I = newPidParams.ki*100;  
            statusToSend.D = newPidParams.kd*100; 
            statusToSend.centerAngle = newPidParams.center_angle;   
            statusToSend.safetyLimits = newPidParams.safety_limits;   // TODO: incluir para enviarlo                 
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

            attitudeControlMotor.motorL = newControlVal.axis_x * -1 * VEL_MAX_CONTROL + newControlVal.axis_y * -1 * VEL_MAX_CONTROL;
            attitudeControlMotor.motorR = newControlVal.axis_x *  VEL_MAX_CONTROL + newControlVal.axis_y * -1 * VEL_MAX_CONTROL;
            
            setVelMotors(attitudeControlMotor.motorL,attitudeControlMotor.motorR);

            if( !attitudeControlMotor.motorL && !attitudeControlMotor.motorR){
                disableMotors();
            }
            else{
                enableMotors();
            }

            printf("CONTROL RECIBIDO: X: %ld, Y: %ld, motorL: %d ,motorR: %d \n",newControlVal.axis_x,newControlVal.axis_y,attitudeControlMotor.motorL,attitudeControlMotor.motorR);
        } 
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

    btInit( DEVICE_BT_NAME );

    mpu6050_init_t mpuConfig = {
        .sclGpio = GPIO_MPU_SCL,
        .sdaGpio = GPIO_MPU_SDA,
        .intGpio = GPIO_MPU_INT,
        .sampleTimeInMs = PERIOD_IMU_MS,
        .accelSensitivity = MPU_ACCEL_SENS_2G,
        .gyroSensitivity = MPU_GYRO_SENS_250,
        .priorityTask = configMAX_PRIORITIES - 1
    };
    mpuInit(mpuConfig);
    readParams = storageReadPidParams();
    printf("center: %f kp: %f , ki: %f , kd: %f,safetyLimits: %f\n",readParams.center_angle,readParams.kp,readParams.ki,readParams.kd,readParams.safety_limits);
    pidInit(readParams);

    setMicroSteps(true);
    motorsInit();
    enableMotors();

    // for(uint16_t i=0;i<1000;i++){
    //     setVelMotors(i,i);

    //     printf("vel: %d\n",i);
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }

    // for(int i=1000;i>0;i--){
    //     setVelMotors(i,i);
    //     printf("vel: %d\n",i);
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }
    // setVelMotors(0,0);

    // while(true){
    //     vTaskDelay(10);
    // }

    xTaskCreatePinnedToCore(imuControlHandler,"Imu Control Task",4096,NULL,IMU_HANDLER_PRIORITY,NULL,IMU_HANDLER_CORE);
    xTaskCreate(updateParams,"Update Params Task",2048,NULL,3,NULL);
    xTaskCreate(attitudeControl,"attitude control Task",2048,NULL,4,NULL);

    setVelMotors(0,0);

    while(1){
        
        if(btIsConnected()){
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
        gpio_set_level(PIN_LED,1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(PIN_LED,0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
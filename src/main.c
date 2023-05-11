#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"

#include "main.h"
#include "comms.h"
#include "PID.h"
// #include "storage_flash.h"

/* Incluyo componentes */
#include "../components/MPU6050/include/MPU6050.h"
#include "../components/TFMINI/include/tfMini.h"
#include "../components/CAN_COMM/include/CAN_COMMS.h"
#include "../components/BT_CLASSIC/include/BT_CLASSIC.h"

extern QueueSetHandle_t newAnglesQueue;                 // Recibo nuevos angulos obtenidos del MPU
QueueHandle_t queueNewPidParams;                 // Recibo nuevos parametros relacionados al pid
QueueSetHandle_t outputMotorQueue;                      // Envio nuevos valores de salida para el control de motores

status_robot_t statusToSend;                            // Estructura que contiene todos los parametros de status a enviar a la app

static void imuControlHandler(void *pvParameters){
    // bool toggle = false;
    float newAngles[3];
    output_motors_t outputMotors;

    outputMotorQueue = xQueueCreate(5,sizeof(output_motors_t));

    pidInit();
    pidSetKs(0,0,0);
    pidSetPointAngle(0);
    pidSetLimits(-100,100);


    while(1){

        if(xQueueReceive(newAnglesQueue,&newAngles,1)){

            statusToSend.pitch = newAngles[AXIS_ANGLE_X];
            statusToSend.roll = newAngles[AXIS_ANGLE_Y];
            statusToSend.yaw = newAngles[AXIS_ANGLE_Z];

            outputMotors.motorL = pidCalculate(newAngles[AXIS_ANGLE_X]);
            outputMotors.motorR=outputMotors.motorL;

            xQueueSend(outputMotorQueue,(void*) &outputMotors,0);                       // TODO: falta recibir los datos de esta cola y enviarlos a los motores
            
            statusToSend.speedL = outputMotors.motorL;
            statusToSend.speedR = outputMotors.motorR;

            // if(toggle){
            //     toggle=0;
            // }
            // else{
            //     toggle=1;
            // }
            // gpio_set_level(PIN_LED,toggle);
        }
    }
}

static void updateParams(void *pvParameters){

    pid_settings_t newPidParams;

    queueNewPidParams = xQueueCreate(1,sizeof(pid_settings_t));

    while (1){
        
        if(xQueueReceive(queueNewPidParams,&newPidParams,0)){
            pidSetKs(newPidParams.kp/100,newPidParams.ki/100,newPidParams.kd/100);
            // pidSetPointAngle( newPidParams.);                                        // TODO: incorporar el centerAngle en la recepcion de parametros del pid
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
}

void app_main() {
    // uint16_t distance = 0;
    uint8_t cont1=0,cont2=0,cont3=0,contNvs = 0;

    gpio_set_direction(PIN_LED , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 0);

    // storageInit();

    // contNvs = storageReadPidParams();
    // printf("Valor leido al iniciar: %d",contNvs);
    // contNvs++;
    // storageWritePidParams(contNvs);

    bt_init();
    mpu_init();
    // tfMiniInit();
    // canInit(GPIO_CAN_TX,GPIO_CAN_RX,UART_PORT_CAN);

    xTaskCreatePinnedToCore(imuControlHandler,"Imu Control Task",2048,NULL,IMU_HANDLER_PRIORITY,NULL,IMU_HANDLER_CORE);
    xTaskCreate(updateParams,"Update Params Task",2048,NULL,6,NULL);
    
    pidEnable();

    while(1){

       printf("angleX: %f , angleY: %f\n",getAngle(AXIS_ANGLE_X),getAngle(AXIS_ANGLE_Y));

        // distance = tfMiniGetDist();
        // printf("angleX: %f  , distance: %d\n",angleX,distance);
         
        // float vNewAngles[3];
        // if(xQueueReceive(newAnglesQueue,&vNewAngles,0)){
        //     printf("angleX: %f \n",vNewAngles[0]);
        // }
        
        if(btIsConnected()){
            cont1++;
            if(cont1>100){
                cont1 =0;
            }

            cont2++;
            if(cont2>4){
                cont2 =0;
            }

            cont3++;
            if(cont3>200){
                cont3 =0;
            }

            // statusToSend={
            //         .header = HEADER_COMMS,
            //         .bat_voltage = 10,//cont1*10,
            //         .bat_percent = cont3,
            //         .batTemp = 100-cont1,
            //         .temp_uc_control = cont1,
            //         .temp_uc_main = 123-cont1,
            //         .speedR = 100,
            //         .speedL = -100,
            //         .pitch = getAngle(AXIS_ANGLE_X),
            //         .roll = getAngle(AXIS_ANGLE_Y),
            //         .yaw = getAngle(AXIS_ANGLE_Z),
            //         .centerAngle = 0,
            //         .P = 10,
            //         .I = 20,
            //         .D = 30,   
            //         .orden_code = 15,
            //         .error_code = cont2,//ERROR_CODE_INIT,
            //         .checksum = 0
            //     };

            statusToSend.header = HEADER_COMMS;
            statusToSend.bat_voltage = 10;
            statusToSend.bat_percent = cont3;
            statusToSend.batTemp = 100-cont1;
            statusToSend.temp_uc_control = cont1;
            statusToSend.temp_uc_main = 123-cont1;
            statusToSend.P = 10;
            statusToSend.I = 20;
            statusToSend.D = 30;   
            sendStatus(statusToSend);
        }
        gpio_set_level(PIN_LED,1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(PIN_LED,0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
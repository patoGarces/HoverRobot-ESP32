#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"


/* Incluyo componentes */
#include "../components/MPU6050/include/MPU6050.h"
#include "../components/TFMINI/include/tfMini.h"
#include "../components/CAN_COMM/include/CAN_COMMS.h"
#include "../components/BT_CLASSIC/include/BT_CLASSIC.h"
#include "comms.h"

#define PIN_LED 27

#define GPIO_CAN_TX     14
#define GPIO_CAN_RX     12
#define UART_PORT_CAN   UART_NUM_1

extern QueueSetHandle_t newAnglesQueue;

void app_main() {
    // uint16_t distance = 0;
    uint8_t cont1=0,cont2=0,cont3=0,cont4=0;

    gpio_set_direction(PIN_LED , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 0);

    bt_init();
    mpu_init();
    // tfMiniInit();
    // canInit(GPIO_CAN_TX,GPIO_CAN_RX,UART_PORT_CAN);
    

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

            status_robot_t status={
                    .header = HEADER_COMMS,
                    .bat_voltage = 10,//cont1*10,
                    .bat_percent = cont3,
                    // .is_charging = 12,//cont3,
                    .batTemp = 100-cont1,
                    .temp_uc_control = cont1,
                    .temp_uc_main = 123-cont1,
                    .speedR = 100,
                    .speedL = -100,
                    .pitch = getAngle(AXIS_ANGLE_X),
                    .roll = getAngle(AXIS_ANGLE_Y),
                    .yaw = getAngle(AXIS_ANGLE_Z),
                    .centerAngle = 0,
                    .P = 10,
                    .I = 20,
                    .D = 30,   
                    .orden_code = 15,
                    .error_code = cont2,//ERROR_CODE_INIT,
                    .checksum = 0
                };
                sendStatus(status);
                // vTaskDelay(pdMS_TO_TICKS(1000));
        }
        gpio_set_level(PIN_LED,1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(PIN_LED,0);
        vTaskDelay(pdMS_TO_TICKS(50));
    
    }
}
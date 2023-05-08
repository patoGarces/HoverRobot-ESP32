#ifndef __MAIN_H__
#define __MAIN_H__
#include "stdint.h"

#define PIN_LED 27

#define GPIO_CAN_TX     14
#define GPIO_CAN_RX     12
#define UART_PORT_CAN   UART_NUM_1


#define IMU_HANDLER_PRIORITY    5
#define IMU_HANDLER_CORE        1


typedef struct{
    int16_t motorR;
    int16_t motorL;
}output_motors_t;



#endif

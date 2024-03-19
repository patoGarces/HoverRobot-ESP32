#ifndef __MAIN_H__
#define __MAIN_H__
#include "stdint.h"

#define PERIOD_IMU_MS   10      

#define PIN_LED         2  //27 en mainBoard
#define PIN_OSCILO      13

// Pinout CAN
#define GPIO_CAN_TX     14
#define GPIO_CAN_RX     12
#define UART_PORT_CAN   UART_NUM_1

// Pinout MPU6050
#define GPIO_MPU_INT        31      
#define GPIO_MPU_SDA        32
#define GPIO_MPU_SCL        33

#define IMU_HANDLER_PRIORITY    6
#define IMU_HANDLER_CORE        1

#define CENTER_ANGLE_MOUNTED    0.00  

typedef struct{
    int16_t motorR;
    int16_t motorL;
}output_motors_t;

enum{
    STATUS_ROBOT_INIT,
    STATUS_ROBOT_DISABLE,
    STATUS_ROBOT_ENABLE,
    STATUS_ROBOT_STABILIZED,
    STATUS_ROBOT_ERROR
};

#endif

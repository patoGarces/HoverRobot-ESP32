#ifndef __MAIN_H__
#define __MAIN_H__
#include "stdint.h"

#define PERIOD_IMU_MS   100//20//2.5      

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

#define GPIO_MOT_L_STEP     21
#define GPIO_MOT_L_DIR      19
#define GPIO_MOT_R_STEP     18
#define GPIO_MOT_R_DIR      5
#define GPIO_MOT_ENABLE     14
#define GPIO_MOT_MICRO_STEP 12

#define MPU_HANDLER_PRIORITY    configMAX_PRIORITIES - 1

#define IMU_HANDLER_PRIORITY    configMAX_PRIORITIES - 2
#define IMU_HANDLER_CORE        1

typedef struct{
    int16_t motorR;
    int16_t motorL;
} output_motors_t;

typedef struct{
    float kp;
    float ki;
    float kd;
    float centerAngle;
    float safetyLimits;
} pid_params_t;

#endif

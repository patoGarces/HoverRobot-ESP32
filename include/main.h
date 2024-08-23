#ifndef __MAIN_H__
#define __MAIN_H__
#include "stdint.h"

// PARA DETECTAR EL ESP32S3: CONFIG_IDF_TARGET_ESP32S3

// #define HARDWARE_PROTOTYPE
#define HARDWARE_S3

#define PERIOD_IMU_MS           100
#define MPU_HANDLER_PRIORITY    5//configMAX_PRIORITIES - 1
#define IMU_HANDLER_PRIORITY    configMAX_PRIORITIES - 2

#define COMMS_HANDLER_CORE      PRO_CPU_NUM     // core 0
#define IMU_HANDLER_CORE        APP_CPU_NUM     // core 1

#define COMM_HANDLER_PRIORITY   configMAX_PRIORITIES - 4

#define PRECISION_DECIMALS_COMMS    100.00              // Precision al convertir la data cruda a float, en este caso 100 = 0.01

#define CANT_PIDS	3

#if defined(HARDWARE_PROTOTYPE) && defined(HARDWARE_S3)
#error Error hardware robot config
#elif !defined(HARDWARE_PROTOTYPE) && !defined(HARDWARE_S3)
#error Error hardware robot config
#endif

#if defined(HARDWARE_PROTOTYPE)
#define DEVICE_BT_NAME          "Balancing robot prototype"
#define PIN_LED             2
#define PIN_OSCILO          27//26

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

#elif defined(HARDWARE_S3)

#define DEVICE_BT_NAME          "Balancing robot S3"
#define PIN_LED         27
#define PIN_OSCILO      3

// Pinout CAN
#define GPIO_CAN_TX     4
#define GPIO_CAN_RX     5
#define UART_PORT_CAN   UART_NUM_2

// Pinout MPU6050
#define GPIO_MPU_INT        9      
#define GPIO_MPU_SDA        18
#define GPIO_MPU_SCL        17

#endif

enum {              // OJO: en sync con App
    PID_ANGLE,
    PID_POS,
    PID_SPEED
};

enum {
    ATT_MODE_ATTI,
    ATT_MODE_POS_CONTROL,
    ATT_MODE_VEL_CONTROL,
};

typedef struct {
    int16_t motorR;
    int16_t motorL;
    uint8_t enable;
} output_motors_t;

// typedef struct {
//     uint8_t indexPid;
//     float kp;
//     float ki;
//     float kd;
//     float centerAngle;
//     float safetyLimits;
// } pid_params_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float setPoint;
} pid_floats_t;

typedef struct {
    int16_t joyAxisX;
    int16_t joyAxisY;
    uint16_t compassYaw;
} direction_control_t;

/**
 * @brief Estructura de datos enviada a la app, contiene settings locales
 */
typedef struct {
    float centerAngle;
    float safetyLimits;
    pid_floats_t pids[CANT_PIDS];
} robot_local_configs_t;

/**
 * @brief Esta estructura de datos generica del robot
 */
typedef struct {
    uint16_t                batVoltage;
    uint16_t                batPercent;
    uint16_t                tempImu;
    // uint16_t                 tempEscs;
    int16_t                 speedR;
    int16_t                 speedL;
    float                   pitch;
    float                   roll;
    float                   yaw;
    int16_t                 speedMeasR;
    int16_t                 speedMeasL;
    float                   posInMetersR;
    float                   posInMetersL;
    float                   distanceInCms;
    direction_control_t     dirControl;
    robot_local_configs_t   localConfig;
    uint16_t                statusCode;
} status_robot_t;

uint32_t ipAddressToUin32(uint8_t ip1,uint8_t ip2,uint8_t ip3,uint8_t ip4);

#endif

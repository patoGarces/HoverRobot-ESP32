#ifndef __MAIN_H__
#define __MAIN_H__
#include "stdint.h"
#include "stdbool.h"

// PARA DETECTAR EL ESP32S3: CONFIG_IDF_TARGET_ESP32S3

#define VERSION_FIRMWARE    0x01

// #define THROTTLE_HOLD_MODE      // DESHABILITA EL MODO STABILIZE

// MCB CONTROL MOTORS MODE:
// #define MCB_TORQUE_MODE // else SPEED_MODE was applied

// #define NAV_CONNECTION_SOCKET true
#define NAV_CONNECTION_SERIAL false

#ifdef MCB_TOQUE_MODE
    #define PERIOD_PID_PRIMARY_MS       5
    #define PERIOD_PID_SECONDARY_MS     20
#else 
    #define PERIOD_PID_PRIMARY_MS       100
    #define PERIOD_PID_SECONDARY_MS     40
#endif

#define MPU_HANDLER_PRIORITY        configMAX_PRIORITIES - 1
#define IMU_HANDLER_PRIORITY        configMAX_PRIORITIES - 2
#define ATTITUDE_HANDLER_PRIORITY   configMAX_PRIORITIES - 3
#define COMM_HANDLER_PRIORITY       configMAX_PRIORITIES - 4

#define COMMS_HANDLER_CORE          PRO_CPU_NUM     // core 0
#define IMU_HANDLER_CORE            APP_CPU_NUM     // core 1

#define PRECISION_DECIMALS_COMMS    100.00              // Precision al convertir la data cruda a float, en este caso 100 = 0.01

#define CANT_PIDS	4

#define TIME_CLEAN_WHEELS_MS        3000
#define SPEED_CLEAN_WHEELS_MS       100

#define MAX_VELOCITY_RPM            1000.00
#define MAX_CYCLES_LIMIT_SPEED      10
#define MAX_VELOCITY_MPS_CONTROL    1.5                 // Velocidad maxima para control en m/s

#define MIN_PITCH_ARMED             1.00
#define MIN_ROLL_ARMED              5.00

#define HOST_SERVER_IP_ESP  "192.168.0.101"
#define CLIENT_IP_APP       "192.168.0.100"
#define CLIENT_IP_RASPI     "192.168.0.102"
#define CLIENT_IP_PC        "192.168.0.103"

#define STREAM_BUFFER_SIZE              500
#define STREAM_BUFFER_LENGTH_TRIGGER    15

#define ESP_WIFI_SSID_AP           "HoverRobotAP"
#define ESP_WIFI_PASS_AP           "12345678"
// AP TENDA
#define ESP_WIFI_SSID_STA           "HoverRobotHubV2"
#define ESP_WIFI_PASS_STA           "12345678"

#define FUSE_ALPHA_YAW      0.9     // Ponderacion Yaw Imu

#ifdef HARDWARE_PROTOTYPE
    #define MAX_ANGLE_CONTROL           15.0
    #define MAX_ROTATION_RATE_CONTROL   100

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

    // Para compatibilidad, no se usa
    #define GPIO_ULTRASONIC_TRIG        -1
    #define GPIO_ULTRASONIC_FRONT_L     -1
    #define GPIO_ULTRASONIC_FRONT_R     -1
    #define GPIO_ULTRASONIC_REAR_L      -1
    #define GPIO_ULTRASONIC_REAR_R      -1

    enum {
        ANGLE_YAW,
        ANGLE_PITCH,
        ANGLE_ROLL
    };

#elif defined(HARDWARE_MAINBOARD) || defined(HARDWARE_SPLITBOARD)
    // #define PIN_LED          27
    #define PIN_OSCILO          32

    #define MAX_ANGLE_CONTROL           10.0
    #define MAX_ROTATION_RATE_CONTROL   40.0

    #ifdef HARDWARE_SPLITBOARD
        #define GPIO_CAN_TX     27
        #define GPIO_CAN_RX     14

    #elif defined(HARDWARE_MAINBOARD)

        // PUERTO 1:
        #define UART_PORT_CAN   UART_NUM_2
        #define GPIO_CAN_TX     14
        #define GPIO_CAN_RX     27

        // PUERTO 2:
        #define UART_PORT_NAV   UART_NUM_1
        #define GPIO_NAV_TX     32
        #define GPIO_NAV_RX     35
        #define UART_NAV_BAUD   115200

        // PUERTO CAN REAL:
        // #define GPIO_CAN_TX     34
        // #define GPIO_CAN_RX     35

    #else
        #error Error hardware mainboard selected
    #endif

    // Pinout MPU6050
    #define GPIO_MPU_INT        12//9      
    #define GPIO_MPU_SDA        26//18
    #define GPIO_MPU_SCL        25//17

    #define GPIO_INPUT_NAV_COMMS_MODE  21

    #define GPIO_ULTRASONIC_TRIG        22
    #define GPIO_ULTRASONIC_FRONT_L     34
    #define GPIO_ULTRASONIC_FRONT_R     39
    #define GPIO_ULTRASONIC_REAR_L      36
    #define GPIO_ULTRASONIC_REAR_R      23

    #define GPIO_LED_RGB_STATUS 13
    #define CANT_LED_STATUS     8

    #define ENABLE_POS_CONTROL      1

    enum {
        ANGLE_YAW,
        ANGLE_PITCH,
        ANGLE_ROLL
    };

#endif

enum {              // OJO: en sync con App
    PID_ANGLE,
    PID_POS,
    PID_SPEED,
    PID_YAW
};

enum {
    ATT_MODE_ATTI,
    ATT_MODE_POS_CONTROL,
    ATT_MODE_MANUAL_CONTROL,
};

// ATENCION: este enum esta emparejado con una enum class en la app, se deben modificar a la vez
enum {
    STATUS_ROBOT_INIT,
    STATUS_ROBOT_DISABLE,
    STATUS_ROBOT_ARMED,
    STATUS_ROBOT_STABILIZED,
    STATUS_ROBOT_CHARGING,
    STATUS_ROBOT_ERROR,
    STATUS_ROBOT_ERROR_LIMIT_SPEED,
    STATUS_ROBOT_ERROR_LIMIT_ANGLE,
    STATUS_ROBOT_ERROR_MCB_CONNECTION,
    STATUS_ROBOT_ERROR_IMU,
    STATUS_ROBOT_ERROR_HALL_L,
    STATUS_ROBOT_ERROR_HALL_R,
    STATUS_ROBOT_ERROR_TEMP,
    STATUS_ROBOT_ERROR_BATTERY,
    STATUS_ROBOT_TEST_MODE
};

typedef struct {
    float kp;
    float ki;
    float kd;
    float setPoint;
} pid_floats_t;

typedef struct {
    int16_t angularVel;             // en rad/s *100 ie: 200 = 2,00 rad/s
    int16_t linearVel;              // en m/s *100 ie: 125 = 1,25 m/s
} velocity_control_t;

/**
 * @brief Estructura de datos enviada a la app, contiene settings locales
 */
typedef struct {
    uint16_t versionFirmware;
    float safetyLimits;
    pid_floats_t pids[CANT_PIDS];
} robot_local_configs_t;

/**
 * @brief Esta estructura de datos generica del robot
 */
typedef struct {
    bool                    networkState;
    uint8_t                 isCharging;
    uint8_t                 isMcbConnected;
    uint16_t                batVoltage;
    uint16_t                batPercent;
    float                   tempImu;
    float                   tempMcb;
    float                   tempMainboard;
    float                   actualPitch;
    float                   actualRoll;
    float                   actualYaw;
    float                   collisionSensors[4];
    int16_t                 speedTargetR;
    int16_t                 speedTargetL;
    int16_t                 speedMeasRpmR;
    int16_t                 speedMeasRpmL;
    float                   posInMetersR;
    float                   posInMetersL;
    int16_t                 currentR;
    int16_t                 currentL;
    float                   actualDistInCms;
    float                   outputYawControl;
    velocity_control_t      dirControl;
    robot_local_configs_t   localConfig;
    uint16_t                statusCode;
} status_robot_t;

uint32_t ipAddressToUin32(uint8_t ip1,uint8_t ip2,uint8_t ip3,uint8_t ip4);

#endif

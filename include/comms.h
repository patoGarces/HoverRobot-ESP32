#ifndef __COMMS_H__
#define __COMMS_H__

#include "stdio.h"
#include "main.h"

#define TIMEOUT_COMMS           15                      // Timeout maximo sin recibir communicacion de la app, en ms /10, ej: 15 = 150ms

#define HEADER_PACKAGE_STATUS           0xAB01          // key a enviar que indica que el paquete enviado a la app es un status
#define HEADER_PACKAGE_CONTROL          0xAB02          // key que indica que el paquete recibido de la app es de control
#define HEADER_PACKAGE_SETTINGS         0xAB03          // key que indica que el paquete recibido de la app es de configuracion
#define HEADER_PACKAGE_COMMAND          0xAB04          // key que indica que el paquete a enviar es un comando

#define HEADER_PACKAGE_LOCAL_CONFIG     0xAB05          // key que indica que el paquete a enviar es una setting local

#define PRECISION_DECIMALS_COMMS    100.00                 // Precision al convertir la data cruda del BLE a float, en este caso 100 = 0.01

enum CommandsToRobot {
    COMMAND_CALIBRATE_IMU,
    COMMAND_ARMED_ROBOT,
    COMMAND_DISARMED_ROBOT,
    COMMAND_VIBRATION_TEST
};

// ATENCION: este enum esta emparejado con una enum class en la app, se deben modificar a la vez
enum {
    STATUS_ROBOT_INIT,
    STATUS_ROBOT_DISABLE,
    STATUS_ROBOT_ARMED,
    STATUS_ROBOT_STABILIZED,
    STATUS_ROBOT_ERROR
};


/**
 * @brief Estructura de datos de configuracion recibida de la app
 */
 typedef struct {
    uint16_t headerPackage;
    uint16_t kp;
    uint16_t ki;
    uint16_t kd;
    int16_t  centerAngle;
    uint16_t safetyLimits;
} pid_settings_raw_t;

/**
 * @brief Estructura de datos de control recibida de la app
 */
 typedef struct {
    uint16_t headerPackage;
    int16_t  axisX;
    int16_t  axisY;
} control_app_raw_t;

/**
 * @brief Estructura de datos de comandos recibida de la app
 */
 typedef struct {
    uint32_t headerPackage;
    int32_t  command;
} command_app_raw_t;

/**
 * @brief Esta estructura de datos es la que se envia a la app android - LEGACY
 */
typedef struct {
    uint16_t batVoltage;
    uint16_t batPercent;
    uint16_t batTemp;
    uint16_t tempImu;
    uint16_t tempEscs;
    int16_t  speedR;
    int16_t  speedL;
    float    pitch;
    float    roll;
    float    yaw;
    int16_t  speedMeasR;
    int16_t  speedMeasL;
    float    posInMetersR;
    float    posInMetersL;
    pid_params_t pidConfigAngle;
    float    setPointAngle;
    uint16_t ordenCode;
    uint16_t statusCode;
} status_robot_t;

/**
 * @brief Esta estructura de datos es la que se envia a la app android
 */
typedef struct {
    uint16_t headerPackage;
    int16_t  speedR;
    int16_t  speedL;
    int16_t  pitch;
    int16_t  roll;
    int16_t  yaw;
    int16_t  setPoint;
    uint16_t centerAngle;
    uint16_t statusCode;
} robot_dynamic_data_t;


/**
 * @brief Estructura de datos enviada a la app, contiene settings locales y c
 */
typedef struct {
    uint16_t headerPackage;
    uint16_t kp;
    uint16_t ki;
    uint16_t kd;
    int16_t  centerAngle;
    uint16_t safetyLimits;
} robot_local_configs_t;

void spp_wr_task_start_up(void);
void spp_wr_task_shut_down(void);
void sendDynamicData(robot_dynamic_data_t status);
void sendLocalConfig(robot_local_configs_t localConfig);
#endif
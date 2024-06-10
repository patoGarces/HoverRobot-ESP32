#ifndef __COMMS_H__
#define __COMMS_H__

#include "stdio.h"
#include "main.h"

#define TIMEOUT_COMMS           15              // Timeout maximo sin recibir communicacion de la app, en ms /10, ej: 15 = 150ms

#define HEADER_COMMS            0xABC0
#define HEADER_TX_KEY_STATUS    0xAB01          // key a enviar que indica que el paquete enviado a la app es un status
#define HEADER_RX_KEY_CONTROL   0xAB02          // key que indica que el paquete recibido de la app es de control
#define HEADER_RX_KEY_SETTINGS  0xAB03          // key que indica que el paquete recibido de la app es de configuracion
#define HEADER_TX_KEY_COMMAND   0xAB04          // key que indica que el paquete a enviar es un comando

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
    uint16_t header;
    uint16_t header_key;
    uint16_t kp;
    uint16_t ki;
    uint16_t kd;
    int16_t  center_angle;
    uint16_t safety_limits;
    uint16_t checksum;
} pid_settings_t;

/**
 * @brief Estructura de datos de control recibida de la app
 */
 typedef struct {
    uint16_t header;
    uint16_t header_key;
    int16_t  axis_x;
    int16_t  axis_y;
    uint16_t checksum;
} control_app_t;

/**
 * @brief Estructura de datos de comandos recibida de la app
 */
 typedef struct {
    uint32_t header;
    uint32_t header_key;
    int32_t  command;
    uint32_t checksum;
} command_app_t;

/**
 * @brief Esta estructura de datos es la que se envia a la app android
 */
typedef struct {
    uint16_t header;
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
    pid_params_t pid;
    float    setPoint;
    uint16_t ordenCode;
    uint16_t statusCode;
    uint16_t checksum;
}status_robot_t;

void spp_wr_task_start_up(void);
void spp_wr_task_shut_down(void);
void sendStatus(status_robot_t status);
#endif
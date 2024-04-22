#ifndef __COMMS_H__
#define __COMMS_H__

#include "stdio.h"
#include "main.h"

#define HEADER_COMMS            0xABC0
#define HEADER_TX_KEY_STATUS    0xAB01          // key a enviar que indica que el paquete enviado a la app es un status
#define HEADER_RX_KEY_CONTROL   0xAB02          // key que indica qe el paquete recibido de la app es de control
#define HEADER_RX_KEY_SETTINGS  0xAB03          // key que indica qe el paquete recibido de la app es de configuracion

/**
 * @brief Estructura de datos de configuracion recibida de la app
 */
 typedef struct{
    uint32_t header;
    uint32_t header_key;
    uint32_t kp;
    uint32_t ki;
    uint32_t kd;
    int32_t  center_angle;
    uint32_t safety_limits;
    uint32_t checksum;
} pid_settings_t;

/**
 * @brief Estructura de datos de control recibida de la app
 */
 typedef struct{
    uint32_t header;
    uint32_t header_key;
    int32_t  axis_x;
    int32_t  axis_y;
    uint32_t checksum;
} control_app_t;

/**
 * @brief Esta estructura de datos es la que se envia a la app android
 */
typedef struct{
    uint16_t header;
    uint16_t bat_voltage;
    uint16_t bat_percent;
    uint16_t batTemp;
    uint16_t temp_uc_control;
    uint16_t temp_uc_main;
    int16_t  speedR;
    int16_t  speedL;
    float    pitch;
    float    roll;
    float    yaw;
    pid_params_t pid;
    float    setPoint;
    uint16_t orden_code;
    uint16_t status_code;
    uint16_t checksum;
}status_robot_t;

void spp_wr_task_start_up(void);
void sendStatus(status_robot_t status);
#endif
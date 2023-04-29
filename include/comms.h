#ifndef __COMMS_H__
#define __COMMS_H__

#include "stdio.h"


 typedef struct{
    uint32_t header;
    uint32_t kp;
    uint32_t ki;
    uint32_t kd;
    uint32_t checksum;
} pidSettings_t;

typedef struct{
    uint16_t bat_voltage;
    uint16_t bat_percent;
    uint16_t temp_uc_control;
    uint16_t temp_uc_main;
    int16_t speedR;
    int16_t speedL;
    uint16_t pitch;
    uint16_t roll;
    uint16_t yaw;
    uint16_t centerAngle;
    uint16_t P;
    uint16_t I;
    uint16_t D;
    uint8_t orden_code;
    uint8_t error_code;
}status_robot_t;

typedef struct{
    uint16_t P;
    uint16_t I;
    uint16_t D;
    uint8_t enable;
    uint8_t orden_code;
    uint8_t error_code;
}rx_bt_app_t;

/**
 * @brief     handler for write and read
 */
typedef void (* spp_wr_task_cb_t) (void *fd);


void spp_read_handle(void * param);
void spp_wr_task_shut_down(void);
void spp_wr_task_start_up(spp_wr_task_cb_t p_cback, int fd);
void sendDato(status_robot_t status);

#endif
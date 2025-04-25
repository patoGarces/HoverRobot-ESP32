#ifndef UTILS_H
#define UTILS_H

#include "stdio.h"
#include "main.h"


#define SPEED_FADE_CONNECTED 1
#define SPEED_FADE_DISCONNECTED 5

typedef struct {
    uint16_t kp;
    uint16_t ki;
    uint16_t kd;
} pid_params_raw_t;

typedef struct {
    int16_t             centerAngle;
    uint16_t            safetyLimits;
    pid_params_raw_t    pids[CANT_PIDS];
} robot_local_configs_raw_t;

pid_params_raw_t convertPidFloatsToRaw(pid_floats_t pidParams);
pid_floats_t convertPidRawToFloats(pid_params_raw_t pidRaw);
robot_local_configs_raw_t convertLocalConfigToRaw(robot_local_configs_t localConfig);
robot_local_configs_t convertLocalConfigToFloat(robot_local_configs_raw_t rawConfig);

void updateStatusLed(uint8_t statusLedRobot);
void statusLedHandler(void *pvParameters);

#endif
#ifndef __STORAGE_FLASH_H__
#define __STORAGE_FLASH_H__

#include "stdio.h"

typedef struct{
    float kp;
    float ki;
    float kd;
    float centerAngle;
}pid_params_t;

void storageInit(void);
void storageWritePidParams(pid_params_t params);
pid_params_t storageReadPidParams(void);

#endif
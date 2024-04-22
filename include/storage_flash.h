#ifndef __STORAGE_FLASH_H__
#define __STORAGE_FLASH_H__

#include "stdio.h"
#include "main.h"

void storageInit(void);
void storageWritePidParams(pid_params_t params);
pid_params_t storageReadPidParams(void);

#endif
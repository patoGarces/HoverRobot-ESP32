#ifndef __STORAGE_FLASH_H__
#define __STORAGE_FLASH_H__

#include "stdio.h"


void storageInit(void);
bool storageWritePidParams(uint16_t cont);
int16_t storageReadPidParams(void);

#endif
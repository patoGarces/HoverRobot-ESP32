#ifndef __PID_H__
#define __PID_H__

#include <stdio.h>
#include "storage_flash.h"
#include "stdbool.h"

void pidInit(pid_params_t params);
void setEnablePid(void);
void setDisablePid(void);
bool getEnablePid(void);
float pidCalculate(float input);
void pidSetPointAngle(float angle);
void pidSetConstants(float KP,float KI,float KD);

typedef struct{
	uint8_t enablePID;
	float output;										//salida del PID, deberia estar entre [-90.00;90.00]
	float input;										//lectura del angulo del eje a equilibrar [-90.00;90.00]
	float kp;											//parametro P
	float ki;											//parametro I
	float kd;											//parametro D
	float setPoint;										//setPoint del PID, normalizado: [-1.00;1.00]
}pid_control_t;

#endif

#ifndef __PID_H__
#define __PID_H__

#include <stdio.h>
#include "storage_flash.h"

void pidInit(pid_params_t params,float angleMin,float angleMax);
void pidEnable(void);
void pidDisable(void);
float pidCalculate(float input);
// void pidSetPointAngle(float angle);
void pidSetLimits(float min,float max);									//con esta funcion seteo los limites máximos y minimos de ambos motores
void pidSetConstants(float KP,float KI,float KD,float targetAngle);

struct PID_n1;

typedef struct{
	
	uint8_t enablePID;
	float output;										//salida del PID, deberia estar entre [-90.00;90.00]
	float input;										//lectura del angulo del eje a equilibrar [-90.00;90.00]
	float kp;											//parametro P
	float ki;											//parametro I
	float kd;											//parametro D
	float set_angle;									//angulo al cual quiero ajustar [-90.00;90.00]
}pid_control_t;

#endif

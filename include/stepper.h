#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "stdint.h"


#define GPIO_MOT_L_STEP     21
#define GPIO_MOT_L_DIR      19
#define GPIO_MOT_R_STEP     18
#define GPIO_MOT_R_DIR      5
#define GPIO_MOT_ENABLE     15


void motorsInit(void);
void enableMotors(void);
void disableMotors(void);
void setVelMotors(int16_t speedL,int16_t speedR);










#endif

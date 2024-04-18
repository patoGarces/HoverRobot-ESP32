#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "stdint.h"

typedef struct {
    uint8_t gpio_mot_l_step;
    uint8_t gpio_mot_l_dir;
    uint8_t gpio_mot_r_step;
    uint8_t gpio_mot_r_dir;
    uint8_t gpio_mot_enable;
    uint8_t gpio_mot_microstepper;
} stepper_config_t;

void motorsInit(stepper_config_t config);
void enableMotors(void);
void disableMotors(void);
void setVelMotors(int16_t speedL,int16_t speedR);
void setMicroSteps(uint8_t fullStep);










#endif

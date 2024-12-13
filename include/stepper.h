#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "stdint.h"

// #define FREQ_MIN  500
// #define FREQ_MAX  30000     // <--- VEL MAX

typedef struct {
    uint8_t gpio_mot_l_step;
    uint8_t gpio_mot_l_dir;
    uint8_t gpio_mot_r_step;
    uint8_t gpio_mot_r_dir;
    uint8_t gpio_mot_enable;
    uint8_t gpio_mot_microstepper;
} stepper_config_t;

typedef struct {
    int32_t absPosL;
    int32_t absPosR;
    uint16_t speedMotL;
    uint16_t speedMotR;
} motors_measurements_t;

void motorsInit(stepper_config_t config);
void setMicroSteps(uint8_t fullStep);
motors_measurements_t getMeasMotors();

#endif

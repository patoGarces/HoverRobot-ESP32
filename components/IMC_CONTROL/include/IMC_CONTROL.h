#ifndef IMC_CONTROL_H
#define IMC_CONTROL_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Integrated Motor Controller

// NEMA 17 1/32
// #define FREQ_MIN  500//1500
// #define FREQ_MAX  30000     // <--- VEL MAX

// // impresora 1/32
#define FREQ_MIN  500
#define FREQ_MAX  5000//7000

// // impresora 1/1
// #define FREQ_MIN  100
// #define FREQ_MAX  500

#define CPU_STEPPER     1

#define SPEED_MODE_TIMER    LEDC_LOW_SPEED_MODE
#define TIMER_MOT_L         LEDC_TIMER_0
#define TIMER_MOT_R         LEDC_TIMER_1

#define CHANNEL_MOT_L       LEDC_CHANNEL_0
#define CHANNEL_MOT_R       LEDC_CHANNEL_1

#define LOW_LIMIT_PCNT -100 // -0x7FFF
#define HIGH_LIMIT_PCNT 100 // 0x7FFF

typedef struct {
    uint8_t gpio_mot_l_step;
    uint8_t gpio_mot_l_dir;
    uint8_t gpio_mot_r_step;
    uint8_t gpio_mot_r_dir;
    uint8_t gpio_mot_enable;
    uint8_t gpio_mot_microstepper;
    QueueHandle_t queueSendControl;
    QueueHandle_t queueReceiveData;
} config_imc_init_t;

typedef struct {
    int32_t absPosL;
    int32_t absPosR;
    uint16_t speedMotL;
    uint16_t speedMotR;
} imc_data_received_t;

typedef struct {
    int16_t motorR;
    int16_t motorL;
    uint8_t enable;
} imc_motor_control_t;

void imcInit(config_imc_init_t config);
void setMicroSteps(uint8_t fullStep);

#endif
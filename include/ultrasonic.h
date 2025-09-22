#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__
#include "stdio.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "main.h"

#define MIN_DISTANCE_ALLOWED_CM         3
#define MAX_DISTANCE_ALLOWED_CM         100

#define MAX_DISTANCE_ALLOWED_IN_MS      MAX_DISTANCE_ALLOWED_CM * 58.00  

#define ALPHA_FILTER        0.7

enum {
    ULTRASONIC_FRONT_LEFT,
    ULTRASONIC_FRONT_RIGHT,
    ULTRASONIC_REAR_LEFT,
    ULTRASONIC_REAR_RIGHT,
};

typedef struct {
    gpio_num_t gpioTrig;
    gpio_num_t gpioSensor[4];
    QueueHandle_t updateQueue;
} ultrasonic_config_t;

void ultrasonicInit(ultrasonic_config_t *config);

#endif
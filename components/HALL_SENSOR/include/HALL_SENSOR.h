#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include "stdio.h"

typedef struct {
    uint8_t gpioSensorU;
    uint8_t gpioSensorV;
    uint8_t gpioSensorW;

} hall_sensors_config_t;



void hallSensorInit(hall_sensors_config_t configHallSensors);


#endif
#ifndef NAV_COMMS_H
#define NAV_COMMS_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "../../../include/main.h"

typedef struct {
    gpio_num_t txPin;
    gpio_num_t rxPin;
    uart_port_t numUart;
    uint32_t baudrate;
    StreamBufferHandle_t xStreamBufferRecv;
    StreamBufferHandle_t xStreamBufferSend;
    QueueHandle_t connectionQueueHandler;
    uint8_t core;
}config_init_nav_t;

void navComms(config_init_nav_t *config);

#endif
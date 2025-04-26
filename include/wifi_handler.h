#ifndef __WIFI_HANDLER_H__
#define __WIFI_HANDLER_H__

#include "esp_wifi.h"
#include "stdint.h"
#include "freertos/queue.h"

void initWifi(char *ssidRed, char* passRed, wifi_mode_t wifiMode, QueueHandle_t connectionStateQueueHandler);

#endif
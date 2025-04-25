#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define TCP_CLIENT_CORE 0

void initTcpClient(char *serverIp,char *ssidRed, char* passRed, QueueHandle_t connectionStateQueueHandler);

#endif
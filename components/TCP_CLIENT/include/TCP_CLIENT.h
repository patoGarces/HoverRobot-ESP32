#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include <stdio.h>

void initTcpClient(char *serverIp);
uint8_t isTcpClientConnected(void);

#endif
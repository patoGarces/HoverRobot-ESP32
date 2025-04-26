#include "include/TCP_CLIENT.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_log.h"

#include "../../../include/comms.h"

#define STREAM_BUFFER_SIZE              500
#define STREAM_BUFFER_LENGTH_TRIGGER    15

#define PORT 8080

/* Usado solo para modo AP*/
#define HOST_IP_ADDR         "192.168.0.100"

StreamBufferHandle_t xStreamBufferReceiver;
StreamBufferHandle_t xStreamBufferSender;

static const char *TAG = "TCP CLIENT";

uint8_t serverClientConnected = false;

static void newConnectionState(QueueHandle_t connectionQueueHandler, bool state) {
    serverClientConnected = state;
    ESP_LOGE(TAG, "new connection state: %d", state);
    if (xQueueOverwrite(connectionQueueHandler, &state) != pdPASS) {
        ESP_LOGE(TAG, "Error al enviar el nuevo estado de connection");
    }
}

static void tcpSocketReceiverTask(void *pvParameters) {
    int socket = *(int *) pvParameters;
    char rx_buffer[128];
    xStreamBufferReset(xStreamBufferReceiver);

    while (serverClientConnected) {

        int len = recv(socket, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len > 0) {
            xStreamBufferSend(xStreamBufferReceiver,rx_buffer,len,1);
        }

        vTaskDelay(pdMS_TO_TICKS(25));
    }
    vTaskDelete(NULL);
}

static void tcpSocketSender(int sock, QueueHandle_t connectionStateQueueHandler) {

    char received_data[100];

    newConnectionState(connectionStateQueueHandler,true);
    xStreamBufferReset(xStreamBufferSender);
    
    while (serverClientConnected) {
        BaseType_t bytesStreamReceived = xStreamBufferReceive(xStreamBufferSender, received_data, sizeof(received_data), 0);

        if (bytesStreamReceived > 1) {
            int errSend = lwip_send(sock, received_data, bytesStreamReceived, 0);
            if (errSend < 0) {
                ESP_LOGE(TAG, "Error conexion perdida 1, errno %d", errno);
                break;
            } 
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

static void tcpClientSocket(void *pvParameters) {
    QueueHandle_t connectionStateQueueHandler = (QueueHandle_t)pvParameters;

    struct sockaddr_in dest_addr = {
        .sin_addr.s_addr = inet_addr(HOST_IP_ADDR),
        .sin_family = AF_INET,
        .sin_port = htons(PORT) 
    };

    while (true) {                        
        int sock =  socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); // ipv4 , x, IPPROTO_IP;
        
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            shutdown(sock, 0);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            ESP_LOGI(TAG, "Socket created, connecting to %s:%d", HOST_IP_ADDR, PORT);

            if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6))) {
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(500));
            } else {
                ESP_LOGI(TAG, "Successfully connected");
                comms_start_up();
                xTaskCreatePinnedToCore(tcpSocketReceiverTask, "tcp_client receiver", 4096, &sock, configMAX_PRIORITIES - 2, NULL, COMMS_HANDLER_CORE);

                // if (connectionStateQueueHandler) {
                //     newConnectionState(connectionStateQueueHandler,true);
                // }
                // xStreamBufferReset(xStreamBufferSender);
                
                // while (serverClientConnected) {
                //     BaseType_t bytesStreamReceived = xStreamBufferReceive(xStreamBufferSender, received_data, sizeof(received_data), 0);

                //     if (bytesStreamReceived > 1) {
                //         int errSend = lwip_send(sock, received_data, bytesStreamReceived, 0);
                //         if (errSend < 0) {
                //             ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                //             break;
                //         } 
                //     }
                //     vTaskDelay(pdMS_TO_TICKS(25));
                // }
                tcpSocketSender(sock, connectionStateQueueHandler);

                // if (connectionStateQueueHandler) {
                    newConnectionState(connectionStateQueueHandler,false);
                // }
                if (sock != -1) {
                    ESP_LOGE(TAG, "Shutting down socket and restarting...");
                    shutdown(sock, 0);
                    close(sock);
                }
            }
        }
    }
    vTaskDelete(NULL);
}

void initTcpClientSocket(QueueHandle_t connectionStateQueueHandler) {
    xStreamBufferSender = xStreamBufferCreate(STREAM_BUFFER_SIZE, STREAM_BUFFER_LENGTH_TRIGGER);
    xStreamBufferReceiver = xStreamBufferCreate(STREAM_BUFFER_SIZE, STREAM_BUFFER_LENGTH_TRIGGER);
    xTaskCreatePinnedToCore(tcpClientSocket, "tcp client task", 4096, connectionStateQueueHandler,configMAX_PRIORITIES - 1, NULL, COMMS_HANDLER_CORE);
}


static void tcpServerSocketTask(void *pvParameters) {

    QueueHandle_t connectionStateQueueHandler = (QueueHandle_t)pvParameters;
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;

    // Calculo demora en al deteccion de perdida de conexion: keepIdle + (keepInterval × keepCount) 
    int keepAlive = 1;
    int keepIdle = 1;           // segundos sin actividad
    int keepInterval = 1;       // periodo de reintento
    int keepCount = 2;          // cantidad de intentos

    struct sockaddr_storage dest_addr;

// #ifdef CONFIG_EXAMPLE_IPV4
    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }
// #endif
// #ifdef CONFIG_EXAMPLE_IPV6
//     if (addr_family == AF_INET6) {
//         struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
//         bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
//         dest_addr_ip6->sin6_family = AF_INET6;
//         dest_addr_ip6->sin6_port = htons(PORT);
//         ip_protocol = IPPROTO_IPV6;
//     }
// #endif

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        close(listen_sock);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
// #ifdef CONFIG_EXAMPLE_IPV4
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
// #endif
// #ifdef CONFIG_EXAMPLE_IPV6
//         if (source_addr.ss_family == PF_INET6) {
//             inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
//         }
// #endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        comms_start_up();
        xTaskCreatePinnedToCore(tcpSocketReceiverTask, "tcp_client receiver", 4096, &sock, configMAX_PRIORITIES - 2, NULL, COMMS_HANDLER_CORE);
        tcpSocketSender(sock, connectionStateQueueHandler);
        newConnectionState(connectionStateQueueHandler, false);
        shutdown(sock, 0);
        close(sock);
    }
}

void initTcpServerSocket(QueueHandle_t connectionStateQueueHandler) {
    xStreamBufferSender = xStreamBufferCreate(STREAM_BUFFER_SIZE, STREAM_BUFFER_LENGTH_TRIGGER);
    xStreamBufferReceiver = xStreamBufferCreate(STREAM_BUFFER_SIZE, STREAM_BUFFER_LENGTH_TRIGGER);
    xTaskCreatePinnedToCore(tcpServerSocketTask, "tcp server task", 4096, connectionStateQueueHandler,configMAX_PRIORITIES - 1, NULL, COMMS_HANDLER_CORE);
}
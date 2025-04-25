#include "include/TCP_CLIENT.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "../../../include/comms.h"

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// STA PROPIO
#define HOST_IP_ADDR                "192.168.4.2"
#define EXAMPLE_ESP_WIFI_CHANNEL    1
#define EXAMPLE_MAX_STA_CONN        2
#define PORT                        8080

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK//WIFI_AUTH_WPA_WPA2_PSK//WIFI_AUTH_WPA2_PSK
#define EXAMPLE_ESP_MAXIMUM_RETRY  30

#define STREAM_BUFFER_SIZE              100
#define STREAM_BUFFER_LENGTH_TRIGGER    3
StreamBufferHandle_t xStreamBufferReceiver;
StreamBufferHandle_t xStreamBufferSender;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "TCP CLIENT";

uint8_t serverClientConnected = false;

static void newConnectionState(QueueHandle_t connectionQueueHandler, bool state) {
    serverClientConnected = state;
    if (xQueueOverwrite(connectionQueueHandler,&state) != pdPASS) {
        ESP_LOGE(TAG, "Error al enviar el nuevo estado de connection");
    }
}

static void tcpClientReceiver(void *pvParameters)
{
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

static void tcpClientSocket(void *pvParameters) {
    QueueHandle_t connectionStateQueueHandler = (QueueHandle_t)pvParameters;
    char received_data[100];

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
                xTaskCreatePinnedToCore(tcpClientReceiver, "tcp_client receiver", 4096, &sock, configMAX_PRIORITIES - 2, NULL, COMMS_HANDLER_CORE);

                if (connectionStateQueueHandler) {
                    newConnectionState(connectionStateQueueHandler,true);
                }
                xStreamBufferReset(xStreamBufferSender);
                
                while (serverClientConnected) {
                    BaseType_t bytesStreamReceived = xStreamBufferReceive(xStreamBufferSender, received_data, sizeof(received_data), 0);

                    if (bytesStreamReceived > 1) {
                        int errSend = lwip_send(sock, received_data, bytesStreamReceived, 0);
                        if (errSend < 0) {
                            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                            break;
                        } 
                    }
                    vTaskDelay(pdMS_TO_TICKS(25));
                }

                if (connectionStateQueueHandler) {
                    newConnectionState(connectionStateQueueHandler,false);
                }
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

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{

    QueueHandle_t connectionStateQueueHandler = (QueueHandle_t)arg;

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);

    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);

        newConnectionState(connectionStateQueueHandler,false); // wifi desconectado
    }
}

void wifi_init_softap(QueueHandle_t connectionStateQueueHandler, const char *ssidRed, const char* passRed) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        connectionStateQueueHandler,
                                                        NULL));


    wifi_config_t wifi_config = {
        .ap = {
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_OPEN,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    const uint8_t lenSSID = strlen((const char *)ssidRed);
    const uint8_t lenPass = strlen((const char *)passRed);

    wifi_config.ap.ssid_len = lenPass;

    strncpy((char *)wifi_config.ap.ssid, ssidRed, lenSSID); 
    wifi_config.ap.ssid[lenSSID] = '\0';
   

    if (lenPass != 0) {
        strncpy((char *)wifi_config.ap.password,passRed, lenPass);
        wifi_config.ap.password[lenPass] = '\0';
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap success -> SSID:%s password:%s channel:%d",
             ssidRed, passRed, EXAMPLE_ESP_WIFI_CHANNEL);
}

static void wifiAndIpEvent(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
 
    static uint8_t wifiSTARetryConnect = 0;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifiSTARetryConnect < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            wifiSTARetryConnect++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        wifiSTARetryConnect = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xTaskCreatePinnedToCore(tcpClientSocket, "tcp_client", 4096, NULL,configMAX_PRIORITIES - 1, NULL, COMMS_HANDLER_CORE);
    }
}

void wifi_init_sta(QueueHandle_t connectionStateQueueHandler, const char *ssidRed, const char* passRed) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifiAndIpEvent,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifiAndIpEvent,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            // .ssid = EXAMPLE_ESP_WIFI_SSID,
            // .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
	     .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    strncpy((char *)wifi_config.sta.ssid,ssidRed, sizeof(wifi_config.sta.ssid) - 1);
    wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
    strncpy((char *)wifi_config.sta.password,passRed, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",ssidRed, passRed);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s, password:%s",ssidRed, passRed);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

void initTcpClient(char *serverIp,char *ssidRed, char* passRed, QueueHandle_t connectionStateQueueHandler) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap(connectionStateQueueHandler, ssidRed, passRed);
    // wifi_init_sta();

    xStreamBufferSender = xStreamBufferCreate(STREAM_BUFFER_SIZE, STREAM_BUFFER_LENGTH_TRIGGER);
    xStreamBufferReceiver = xStreamBufferCreate(STREAM_BUFFER_SIZE, STREAM_BUFFER_LENGTH_TRIGGER);

    xTaskCreatePinnedToCore(tcpClientSocket, "tcp client task", 4096, connectionStateQueueHandler,configMAX_PRIORITIES - 1, NULL, COMMS_HANDLER_CORE);
}
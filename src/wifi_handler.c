#include "wifi_handler.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/ip4_addr.h"
#include "esp_log.h"
#include <string.h>
#include "main.h"

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define EXAMPLE_ESP_WIFI_CHANNEL 1
#define EXAMPLE_MAX_STA_CONN 2

#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "WIFI HANDLER";

static void updateConnectionState(QueueHandle_t stateQueueHandler, bool state) {
    if (xQueueOverwrite(stateQueueHandler, &state) != pdPASS) {
        ESP_LOGE(TAG, "Error al enviar el nuevo estado de network");
    }
}

/* 
 * Metodo para asignar una ip fija al propio ESP 
 */
void configureAPWithStaticIP() {
    esp_netif_ip_info_t ip_info;

    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    // Detener DHCP server
    esp_netif_dhcps_stop(netif);

    ip4_addr_t temp_ip;
    ip4addr_aton("192.168.0.101", &temp_ip);
    ip_info.ip.addr = temp_ip.addr;
    ip_info.gw.addr = temp_ip.addr;
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    // Configurar IP estática
    // IP4_ADDR(&ip_info.ip, 192, 168, 0, 101);        // IP del ESP32
    // IP4_ADDR(&ip_info.gw, 192, 168, 0, 101);         // Gateway igual
    // IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    

    esp_netif_set_ip_info(netif, &ip_info);

    // Reiniciar DHCP server
    esp_netif_dhcps_start(netif);
}

static void wifi_event_handler_ap(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {

    QueueHandle_t networkStateHandler = (QueueHandle_t)arg;

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
        updateConnectionState(networkStateHandler, true); // wifi conectado
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
        updateConnectionState(networkStateHandler, false); // wifi desconectado
    }
}

void wifi_init_softap(QueueHandle_t networkStateHandler, const char *ssidRed, const char *passRed) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler_ap,
                                                        networkStateHandler,
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

    wifi_config.ap.ssid_len = lenSSID;

    strncpy((char *)wifi_config.ap.ssid, ssidRed, lenSSID);
    wifi_config.ap.ssid[lenSSID] = '\0';

    if (lenPass != 0) {
        strncpy((char *)wifi_config.ap.password, passRed, lenPass);
        wifi_config.ap.password[lenPass] = '\0';
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    configureAPWithStaticIP();

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap success -> SSID:%s password:%s channel:%d",
             ssidRed, passRed, EXAMPLE_ESP_WIFI_CHANNEL);
}

static void wifi_event_handler_sta(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data) {

    QueueHandle_t networkStateHandler = (QueueHandle_t)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        updateConnectionState(networkStateHandler, false);
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        updateConnectionState(networkStateHandler, true);
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(QueueHandle_t networkStateQueueHandler, const char *ssidRed, const char *passRed) {
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
                                                        &wifi_event_handler_sta,
                                                        networkStateQueueHandler,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler_sta,
                                                        networkStateQueueHandler,
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

    // copio ssid y password:
    strncpy((char *)wifi_config.sta.ssid, ssidRed, sizeof(wifi_config.sta.ssid) - 1);
    wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
    strncpy((char *)wifi_config.sta.password, passRed, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

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
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", ssidRed, passRed);
    }
    else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s, password:%s", ssidRed, passRed);
    }
    else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void initWifi(char *ssidRed, char *passRed, wifi_mode_t wifiMode, QueueHandle_t networkStateQueueHandler) {

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (wifiMode == WIFI_MODE_AP) {
        wifi_init_softap(networkStateQueueHandler, ssidRed, passRed);
    }
    else {
        wifi_init_sta(networkStateQueueHandler, ssidRed, passRed);
    }
}
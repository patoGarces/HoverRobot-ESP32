#include "utils.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

/* Incluyo componentes */
#include "../components/WS2812/esp_ws28xx.h"

QueueHandle_t statusLedQueue;

static const CRGB STATUS_COLOR_INIT = {.r = 125, .g = 125, .b = 10};
static const CRGB STATUS_COLOR_DISABLE = {.r = 60, .g = 60, .b = 60};
static const CRGB STATUS_COLOR_ARMED = {.r = 0, .g = 75, .b = 68};
static const CRGB STATUS_COLOR_STABILIZED = {.r = 10, .g = 10, .b = 130};
static const CRGB STATUS_COLOR_ERROR = {.r = 120, .g = 0, .b = 0};
// static const CRGB STATUS_COLOR_OFF =s {.r = 0, .g = 0, .b = 0};

typedef struct {
    CRGB ws2812_color;
    uint8_t isTcpConnected;
} status_led_state_t;


pid_params_raw_t convertPidFloatsToRaw(pid_floats_t pidParams) {

    pid_params_raw_t pidRaw = {
        .kp = pidParams.kp * PRECISION_DECIMALS_COMMS,
        .ki = pidParams.ki * PRECISION_DECIMALS_COMMS,
        .kd = pidParams.kd * PRECISION_DECIMALS_COMMS,
    };
    return pidRaw;
}

pid_floats_t convertPidRawToFloats(pid_params_raw_t pidRaw) {

    pid_floats_t pidParams = {
        .kp = pidRaw.kp / PRECISION_DECIMALS_COMMS,
        .ki = pidRaw.ki / PRECISION_DECIMALS_COMMS,
        .kd = pidRaw.kd / PRECISION_DECIMALS_COMMS,
    };
    return pidParams;
}

/*
 * Convierto los parametros raw a float
*/
robot_local_configs_raw_t convertLocalConfigToRaw(robot_local_configs_t localConfig) {

    pid_params_raw_t pids[CANT_PIDS];
    for (uint8_t i=0;i<CANT_PIDS;i++) {
        pids[i] = convertPidFloatsToRaw(localConfig.pids[i]);
    }

    robot_local_configs_raw_t pidRaw = {
        .safetyLimits = localConfig.safetyLimits * PRECISION_DECIMALS_COMMS,
    };
    memcpy(pidRaw.pids,pids,sizeof(pids));

    return pidRaw;
}

/*
 * Convierto a float los parametros en raw
*/
robot_local_configs_t convertLocalConfigToFloat(robot_local_configs_raw_t rawConfig) {

    pid_floats_t pids[CANT_PIDS];
    for (uint8_t i=0;i<CANT_PIDS;i++) {
        pids[i] = convertPidRawToFloats(rawConfig.pids[i]);
    }

    robot_local_configs_t localConfigParams = {
        .safetyLimits = rawConfig.safetyLimits * PRECISION_DECIMALS_COMMS,
    };
    memcpy(localConfigParams.pids,pids,sizeof(pids));

    return localConfigParams;
}

void statusLedHandler(void *pvParameters) {
    CRGB* ws2812_buffer;
    status_led_state_t newLedStatus;
    const uint8_t MAX_BRIGHTNESS = 100;
    uint8_t fadeIntensity = 0, fadeSpeed = SPEED_FADE_DISCONNECTED, socketClientsConnected = 0;
    bool fadeIn = true;
    const char *TAG = "StatusLedHandler";

    QueueHandle_t connectionStateQueueHandler = (QueueHandle_t)pvParameters;

    newLedStatus.ws2812_color = STATUS_COLOR_INIT;

    ESP_LOGI(TAG,"statusLedHandler init");
    ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_init(GPIO_LED_RGB_STATUS, WS2812B, CANT_LED_STATUS, &ws2812_buffer));

    statusLedQueue = xQueueCreate(1, sizeof(status_led_state_t));
    if (statusLedQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create status LED queue");
    }

    while(true) {
        xQueueReceive(statusLedQueue,&newLedStatus,0);
        xQueuePeek(connectionStateQueueHandler, &socketClientsConnected, 0); 
        fadeSpeed = (socketClientsConnected > 0) ? SPEED_FADE_CONNECTED : SPEED_FADE_DISCONNECTED;

        if (fadeIn) {
            fadeIntensity += fadeSpeed;
            if (fadeIntensity > MAX_BRIGHTNESS) {
                fadeIn = false;
            }
        } else {
            if (fadeIntensity > fadeSpeed) {
                fadeIntensity -= fadeSpeed;
            } else {
                fadeIn = true;
            }
        }

        for (int i = 0; i < CANT_LED_STATUS; i++) {
            ws2812_buffer[i].r = (newLedStatus.ws2812_color.r * fadeIntensity) / MAX_BRIGHTNESS;
            ws2812_buffer[i].g = (newLedStatus.ws2812_color.g * fadeIntensity) / MAX_BRIGHTNESS;
            ws2812_buffer[i].b = (newLedStatus.ws2812_color.b * fadeIntensity) / MAX_BRIGHTNESS;
        }
        ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_update());

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}

void updateStatusLed(uint8_t statusLedColor) {
    status_led_state_t statusLedState;
    CRGB color;
    statusLedState.isTcpConnected = false;//isTcpConnected;

    switch(statusLedColor) {
        case STATUS_ROBOT_INIT:
            color = STATUS_COLOR_INIT;
        break;
        case STATUS_ROBOT_DISABLE:
            color = STATUS_COLOR_DISABLE;
        break;
        case STATUS_ROBOT_ARMED:
            color = STATUS_COLOR_ARMED;
        break;
        case STATUS_ROBOT_STABILIZED:
            color = STATUS_COLOR_STABILIZED;
        break;
        case STATUS_ROBOT_ERROR:                // TODO: diferenciar por color o blink
        case STATUS_ROBOT_ERROR_BATTERY:
        case STATUS_ROBOT_ERROR_HALL_L:
        case STATUS_ROBOT_ERROR_HALL_R:
        case STATUS_ROBOT_ERROR_TEMP:
        case STATUS_ROBOT_ERROR_IMU:
        case STATUS_ROBOT_ERROR_LIMIT_SPEED:
        case STATUS_ROBOT_ERROR_MCB_CONNECTION:
            color = STATUS_COLOR_ERROR;
        break;

        default:
            ESP_LOGE("updateStatusLed","Error status color unknown");
        break;
    };

    statusLedState.ws2812_color = color;
    xQueueSend(statusLedQueue,&statusLedState,1);
}
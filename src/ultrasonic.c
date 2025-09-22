#include "ultrasonic.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "soc/gpio_periph.h"
#include "math.h"

ultrasonic_config_t sensorConfig;
static volatile int64_t distanceMeasureInUs[4];
static volatile int64_t initTrigSensors[4];

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR interruptEcho(void *arg) {
    uint8_t numSensor = (uint32_t) arg;

    portENTER_CRITICAL_ISR(&mux); // Simil a un Mutex para no permitir que interrumpa otra ISR con igual o mayor prioridad
    if (gpio_get_level(sensorConfig.gpioSensor[numSensor])) {
        if (initTrigSensors[numSensor] == 0) {
            initTrigSensors[numSensor] = esp_timer_get_time();
        }
    } else {
        if (initTrigSensors[numSensor] > 0) {
            distanceMeasureInUs[numSensor] = esp_timer_get_time() - initTrigSensors[numSensor];
            
        }
        initTrigSensors[numSensor] = 0;
    }
    portEXIT_CRITICAL_ISR(&mux);
}

static void ultrasonicHandler() {
    float distanceMeasureInCm[4];
    float lastValues[4] = { 0, 0, 0, 0};

    while(true) {
        gpio_set_level(sensorConfig.gpioTrig, 1);
        uint64_t initialTime = esp_timer_get_time(); 
        while((esp_timer_get_time() - initialTime) < 10);
        gpio_set_level(sensorConfig.gpioTrig, 0);  

        vTaskDelay(100);

        for (uint8_t i=0; i<4; i++) {
            float newDistanceInCm = distanceMeasureInUs[i] / 58.00;
            if (newDistanceInCm > MIN_DISTANCE_ALLOWED_CM && newDistanceInCm < MAX_DISTANCE_ALLOWED_CM) {
                distanceMeasureInCm[i] = ALPHA_FILTER * newDistanceInCm + (1.0 - ALPHA_FILTER) * lastValues[i];
                lastValues[i] = distanceMeasureInCm[i];
            } else {
                distanceMeasureInCm[i] = -1;
            }
            distanceMeasureInUs[i] = 0;
        }

        xQueueSend(sensorConfig.updateQueue, distanceMeasureInCm, 0);
    }
}

void ultrasonicInit(ultrasonic_config_t *config) {

    sensorConfig = *config;

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[sensorConfig.gpioTrig], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[sensorConfig.gpioSensor[ULTRASONIC_FRONT_LEFT]], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[sensorConfig.gpioSensor[ULTRASONIC_FRONT_RIGHT]], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[sensorConfig.gpioSensor[ULTRASONIC_REAR_LEFT]], PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[sensorConfig.gpioSensor[ULTRASONIC_REAR_RIGHT]], PIN_FUNC_GPIO);

    gpio_config_t configGpio = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << GPIO_ULTRASONIC_TRIG,
        .pull_down_en = false,
        .pull_up_en = false
    };
    gpio_config(&configGpio);

    for (uint8_t i=0; i<4; i++) {
        configGpio.mode = GPIO_MODE_INPUT;
        configGpio.pin_bit_mask = 1ULL << sensorConfig.gpioSensor[i];
        configGpio.pull_down_en = false;
        configGpio.pull_up_en = false;
        configGpio.intr_type = GPIO_INTR_ANYEDGE;
        gpio_config(&configGpio);
    }

    gpio_install_isr_service(0);//ESP_INTR_FLAG_LEVEL1);        // TODO: revisar

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_ULTRASONIC_FRONT_L, interruptEcho, (void*) ULTRASONIC_FRONT_LEFT);
    gpio_isr_handler_add(GPIO_ULTRASONIC_FRONT_R, interruptEcho, (void*) ULTRASONIC_FRONT_RIGHT);
    gpio_isr_handler_add(GPIO_ULTRASONIC_REAR_L, interruptEcho, (void*) ULTRASONIC_REAR_LEFT);
    gpio_isr_handler_add(GPIO_ULTRASONIC_REAR_R, interruptEcho, (void*) ULTRASONIC_REAR_RIGHT);

    // TODO: mover el core y el priority
    xTaskCreatePinnedToCore(ultrasonicHandler, "Ultrasonic sensor task", 2048, NULL, configMAX_PRIORITIES -6, NULL, IMU_HANDLER_CORE);
}
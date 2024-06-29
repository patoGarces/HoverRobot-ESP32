#include "include/HALL_SENSOR.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static hall_sensors_config_t hallSensorConfig;

#define CW             1			// Assign a value to represent clock wise rotation
#define CCW           -1

int32_t pulseCount = 0;				// Integer variable to store the pulse count

uint8_t HSU_Val = 0;
uint8_t HSV_Val = 0;
uint8_t HSW_Val = 0;

static void hallSensorsHandler(void *pvParameters) {

    int16_t lastValue = 0;
    HSU_Val = gpio_get_level(hallSensorConfig.gpioSensorU);
    HSV_Val = gpio_get_level(hallSensorConfig.gpioSensorV);
    HSW_Val = gpio_get_level(hallSensorConfig.gpioSensorW);
      
    while(1) { 
        if(pulseCount != lastValue) {
            printf("pulseCount: %ld\n",pulseCount);
            lastValue = pulseCount;
        }
        vTaskDelay(50);
    }
}

static void IRAM_ATTR hallSensorISR(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;

    if(gpio_num == hallSensorConfig.gpioSensorU) {
        HSU_Val = gpio_get_level(hallSensorConfig.gpioSensorU);
        HSW_Val = gpio_get_level(hallSensorConfig.gpioSensorW);
        // startTime = millis();
        int8_t direct = (HSU_Val == HSW_Val) ? CW : CCW;
        pulseCount = pulseCount + direct;
        // pulseTimeU = startTime - prevTime;				
        // AvPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV)/3);		
        // PPM = (1000 / AvPulseTime) * 60;					
        // RPM = PPM / 90;
        // prevTime = startTime;
    }
    else if (gpio_num == hallSensorConfig.gpioSensorV) {
        HSV_Val = gpio_get_level(hallSensorConfig.gpioSensorV);
        HSU_Val = gpio_get_level(hallSensorConfig.gpioSensorU);
        // startTime = millis();
        int8_t direct = (HSV_Val == HSU_Val) ? CW : CCW;
        pulseCount = pulseCount + direct;
        // pulseTimeV = startTime - prevTime;				
        // AvPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV)/3);		
        // PPM = (1000 / AvPulseTime) * 60;					
        // RPM = PPM / 90;
        // prevTime = startTime;
    }
    else if (gpio_num == hallSensorConfig.gpioSensorW) {
        HSW_Val = gpio_get_level(hallSensorConfig.gpioSensorW);
        HSV_Val = gpio_get_level(hallSensorConfig.gpioSensorV);
        // startTime = millis();						// Set startTime to current microcontroller elapsed time value
        int8_t direct = (HSW_Val == HSV_Val) ? CW : CCW;			// Determine rotation direction (ternary if statement)
        pulseCount = pulseCount + direct;				// Add 1 to the pulse count
        // pulseTimeW = startTime - prevTime;				// Calculate the current time between pulses
        // AvPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV)/3);	// Calculate the average time time between pulses
        // PPM = (1000 / AvPulseTime) * 60;					// Calculate the pulses per min (1000 millis in 1 second)
        // RPM = PPM / 90;						// Calculate revs per minute based on 90 pulses per rev
        // prevTime = startTime;						// Remember the start time for the next interrupt
    }
    else {
        printf("ERROR DETECCION GPIO ISR\n");
    }
}

void hallSensorInit(hall_sensors_config_t configHallSensors) {
    
    hallSensorConfig = configHallSensors;

    gpio_config_t gpioConfig = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1 << configHallSensors.gpioSensorU,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&gpioConfig);

    gpioConfig.pin_bit_mask = 1 << configHallSensors.gpioSensorV;
    gpio_config(&gpioConfig);

    gpioConfig.pin_bit_mask = 1 << configHallSensors.gpioSensorW;
    gpio_config(&gpioConfig);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(configHallSensors.gpioSensorU, hallSensorISR, (uint8_t*)configHallSensors.gpioSensorU);
    gpio_isr_handler_add(configHallSensors.gpioSensorV, hallSensorISR, (uint8_t*)configHallSensors.gpioSensorV);
    gpio_isr_handler_add(configHallSensors.gpioSensorW, hallSensorISR, (uint8_t*)configHallSensors.gpioSensorW);

    xTaskCreate(hallSensorsHandler,"Hall sensors handler",2048,NULL,5,NULL);
}
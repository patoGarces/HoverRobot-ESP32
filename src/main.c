#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"

/* Incluyo componentes */
#include "../components/MPU6050/include/MPU6050.h"


void app_main() {
    float angleX = 0.00;

    mpu_init();

    while(1){
        angleX = getAngle(AXIS_ANGLE_X);
        printf("angleX: %f",angleX);
        vTaskDelay(pdMS_TO_TICKS(100));
    
    }
}
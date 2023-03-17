#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"

/* Incluyo componentes */
#include "../components/MPU6050/include/MPU6050.h"
#include "../components/TFMINI/include/tfMini.h"


void app_main() {
    float angleX = 0.00;
    uint16_t distance = 0;

    mpu_init();
    tfMiniInit();

    while(1){

        angleX = getAngle(AXIS_ANGLE_X);
        distance = tfMiniGetDist();

        printf("angleX: %f  , distance: %d",angleX,distance);
        vTaskDelay(pdMS_TO_TICKS(100));
    
    }
}
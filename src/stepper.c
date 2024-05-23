#include "stepper.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "driver/ledc.h"
#include "freertos/queue.h"

#include "../include/main.h"

// NEMA 17 1/32
#define FREQ_MIN  1500
#define FREQ_MAX  30000     // <--- VEL MAX

// // impresora 1/32
// #define FREQ_MIN  3500
// #define FREQ_MAX  7000

// // impresora 1/1
// #define FREQ_MIN  100
// #define FREQ_MAX  500

#define CPU_STEPPER     1

#define SPEED_MODE_TIMER    LEDC_HIGH_SPEED_MODE
#define TIMER_MOT_L         LEDC_TIMER_0
#define TIMER_MOT_R         LEDC_TIMER_1

#define CHANNEL_MOT_L       LEDC_CHANNEL_0
#define CHANNEL_MOT_R       LEDC_CHANNEL_1

extern QueueSetHandle_t queueMotorControl; 
static stepper_config_t configInit;

static void setVelMotors(int16_t speedL,int16_t speedR);
static void setEnableMotors(uint8_t enable);

static void controlHandler(void *pvParameters) {

    output_motors_t newVel;
    while(true) {
        if (xQueueReceive(queueMotorControl,&newVel,pdMS_TO_TICKS(1))) {
            if (newVel.enable) {
                setVelMotors(newVel.motorL,newVel.motorR);
            }
            else {
                setVelMotors(0,0);
            }
            setEnableMotors(newVel.enable);                         // TODO: esto se va a llamar continuamente
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void motorsInit(stepper_config_t config) {

    configInit = config;
    /* seteo pines de salida de steps */
    gpio_config_t pinesMotor={
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ( 1 << config.gpio_mot_l_dir),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&pinesMotor);

    pinesMotor.pin_bit_mask = (1 << config.gpio_mot_r_dir);
    gpio_config(&pinesMotor);

    /* seteo pines de salida de enable*/
    pinesMotor.pin_bit_mask = (1 << config.gpio_mot_enable);
    gpio_config(&pinesMotor);

    pinesMotor.pin_bit_mask = (1 << config.gpio_mot_microstepper);
    gpio_config(&pinesMotor);

    ledc_timer_config_t timerConfig = {
        .speed_mode = SPEED_MODE_TIMER,
        .timer_num = TIMER_MOT_L,
        .clk_cfg = LEDC_USE_REF_TICK,
        .duty_resolution = LEDC_TIMER_4_BIT,
        .freq_hz = FREQ_MIN,
    };

    ledc_timer_config(&timerConfig);            // Timer config motor L

    timerConfig.timer_num = TIMER_MOT_R;
    ledc_timer_config(&timerConfig);            // Timer config motor R

    ledc_channel_config_t channelConfig = {
        .gpio_num = config.gpio_mot_l_step,
        .speed_mode = SPEED_MODE_TIMER,
        .channel = CHANNEL_MOT_L,
        .intr_type =  LEDC_INTR_DISABLE,
        .timer_sel = TIMER_MOT_L,
        .duty = 0,                              // 0 -> 0% de duty, 8 -> 50% de duty
        .hpoint = 0,
    };
    ledc_channel_config(&channelConfig);        // Channel config motor L

    channelConfig.gpio_num = config.gpio_mot_r_step;
    channelConfig.channel = CHANNEL_MOT_R;
    channelConfig.timer_sel = TIMER_MOT_R;

    ledc_channel_config(&channelConfig);        // Channel config motor R

    ledc_timer_pause(SPEED_MODE_TIMER,TIMER_MOT_L);
    ledc_timer_pause(SPEED_MODE_TIMER,TIMER_MOT_R);

    setEnableMotors(false);
    setVelMotors(0,0);
    queueMotorControl = xQueueCreate(1,sizeof(output_motors_t));
    xTaskCreate(controlHandler,"motor control handler task",4096,NULL,5,NULL);
}

static void setEnableMotors(uint8_t enable) {
    gpio_set_level(configInit.gpio_mot_enable,!enable);
    if (enable) {
        ledc_timer_resume(SPEED_MODE_TIMER,TIMER_MOT_L);
        ledc_timer_resume(SPEED_MODE_TIMER,TIMER_MOT_R);
    }
    else {
        ledc_timer_pause(SPEED_MODE_TIMER,TIMER_MOT_L);
        ledc_timer_pause(SPEED_MODE_TIMER,TIMER_MOT_R);
    }
    // printf("enable motors: %d\n",enable);
}

void setVelMotors(int16_t speedL,int16_t speedR) {
    uint32_t timerFreqL=0,timerFreqR=0;

    if (!speedL) {
        ledc_set_duty(SPEED_MODE_TIMER,CHANNEL_MOT_L,0);
        ledc_update_duty(SPEED_MODE_TIMER,CHANNEL_MOT_L);
        // printf("MotorL pause\n");
    }
    else if (speedL <= 1000 && speedL >= -1000) {
        ledc_set_duty(SPEED_MODE_TIMER,CHANNEL_MOT_L,8);
        ledc_update_duty(SPEED_MODE_TIMER,CHANNEL_MOT_L);

        timerFreqL = FREQ_MIN + (abs(speedL)*((FREQ_MAX-FREQ_MIN)/1000));
        ledc_set_freq(SPEED_MODE_TIMER,TIMER_MOT_L,timerFreqL);
        
        gpio_set_level(configInit.gpio_mot_l_dir,speedL < 0);
    }
    else {
        setEnableMotors(false);
        printf("Error speedMotors -> DisableMotor\n");
    }

    if(!speedR) {
        ledc_set_duty(SPEED_MODE_TIMER,CHANNEL_MOT_R,0);
        ledc_update_duty(SPEED_MODE_TIMER,CHANNEL_MOT_R);
        // printf("MotorR pause\n");
    }
    else if (speedR <= 1000 && speedR >= -1000) {
        ledc_set_duty(SPEED_MODE_TIMER,CHANNEL_MOT_R,8);
        ledc_update_duty(SPEED_MODE_TIMER,CHANNEL_MOT_R);

        timerFreqR = FREQ_MIN + (abs(speedR)*((FREQ_MAX-FREQ_MIN)/1000));
        ledc_set_freq(SPEED_MODE_TIMER,TIMER_MOT_R,timerFreqR);
        gpio_set_level(configInit.gpio_mot_r_dir,speedR < 0);
    }
    else {
        setEnableMotors(false);
        printf("Error speedMotors -> DisableMotor\n");
    }
    // printf("speedL: %d,speedR: %d, freqTimerL: %ld, freqTimerR: %ld\n",speedL,speedR,timerFreqL,timerFreqR);
}

void setMicroSteps(uint8_t fullStep) {
    gpio_set_level(configInit.gpio_mot_microstepper, fullStep);
}
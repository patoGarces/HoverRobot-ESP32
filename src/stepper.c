#include "stepper.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "driver/ledc.h"

#define FREQ_MIN  200
#define FREQ_MAX  1500

#define CPU_STEPPER     1

#define SPEED_MODE_TIMER LEDC_LOW_SPEED_MODE
#define TIMER_MOT_L  LEDC_TIMER_0
#define TIMER_MOT_R  LEDC_TIMER_1

#define CHANNEL_MOT_L  LEDC_CHANNEL_0
#define CHANNEL_MOT_R  LEDC_CHANNEL_1

uint8_t enableVelMot=0;

void motorsInit(void){
    /* seteo pines de salida de steps */
    gpio_config_t pinesMotor={
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ( 1 << GPIO_MOT_L_DIR),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&pinesMotor);

    pinesMotor.pin_bit_mask = (1 << GPIO_MOT_R_DIR);
    gpio_config(&pinesMotor);

    /* seteo pines de salida de enable*/
    pinesMotor.pin_bit_mask = (1 << GPIO_MOT_ENABLE);
    gpio_config(&pinesMotor);

    gpio_set_level(GPIO_MOT_ENABLE,1);

    ledc_timer_config_t timerConfig={
        .speed_mode = SPEED_MODE_TIMER,
        .timer_num = TIMER_MOT_L,
        .clk_cfg = LEDC_USE_REF_TICK,
        .duty_resolution = LEDC_TIMER_4_BIT,
        .freq_hz = 10000,
    };

    ledc_timer_config(&timerConfig);            // Timer config motor L

    timerConfig.timer_num = TIMER_MOT_R;
    ledc_timer_config(&timerConfig);            // Timer config motor R

    ledc_channel_config_t channelConfig = {
        .gpio_num = GPIO_MOT_L_STEP,
        .speed_mode = SPEED_MODE_TIMER,
        .channel = CHANNEL_MOT_L,
        .intr_type =  LEDC_INTR_DISABLE,
        .timer_sel = TIMER_MOT_L,
        .duty = 8,                              // 50% de duty
        .hpoint = 0,
    };
    ledc_channel_config(&channelConfig);        // Channel config motor L

    channelConfig.gpio_num = GPIO_MOT_R_STEP;
    channelConfig.channel = CHANNEL_MOT_R;
    channelConfig.timer_sel = TIMER_MOT_R;

    ledc_channel_config(&channelConfig);        // Channel config motor R

    // ledc_set_freq(SPEED_MODE_TIMER,TIMER_MOT_L,2000);

    ledc_timer_pause(SPEED_MODE_TIMER,TIMER_MOT_L);
    ledc_timer_pause(SPEED_MODE_TIMER,TIMER_MOT_R);
}


void enableMotors(void){
    gpio_set_level(GPIO_MOT_ENABLE,0);
    ledc_timer_resume(SPEED_MODE_TIMER,TIMER_MOT_L);
    ledc_timer_resume(SPEED_MODE_TIMER,TIMER_MOT_R);
}
void disableMotors(void){
    gpio_set_level(GPIO_MOT_ENABLE,1);
    ledc_timer_pause(SPEED_MODE_TIMER,TIMER_MOT_L);
    ledc_timer_pause(SPEED_MODE_TIMER,TIMER_MOT_R);
}

void setVelMotors(int16_t speedL,int16_t speedR){
    uint32_t timerFreqL=0,timerFreqR=0;

    if(speedL <= 1000 && speedL >= -1000){

        timerFreqL = FREQ_MIN + (abs(speedL)*((FREQ_MAX-FREQ_MIN)/1000));
        ledc_set_freq(SPEED_MODE_TIMER,TIMER_MOT_L,timerFreqL);
        // printf("speedL: %d , freqL :%ld\n",speedL,timerFreqL);
        
        if(speedL < 0){
            gpio_set_level(GPIO_MOT_L_DIR,1);
        }
        else{
            gpio_set_level(GPIO_MOT_L_DIR,0);
        }
    }
    else{
 //       enableVelMot=0;
    }

    if(speedR <= 1000 && speedR >= -1000){
        timerFreqR = FREQ_MIN + (abs(speedL)*((FREQ_MAX-FREQ_MIN)/1000));
        ledc_set_freq(SPEED_MODE_TIMER,TIMER_MOT_R,timerFreqR);
        if(speedR < 0){
            gpio_set_level(GPIO_MOT_R_DIR,0);
        }
        else{
            gpio_set_level(GPIO_MOT_R_DIR,1);
        }
    }
    else{
//        enableVelMot=0;
    }

    // printf("Vel: %d , enableVel: %d\n",pwmr,enableVelMot);
}

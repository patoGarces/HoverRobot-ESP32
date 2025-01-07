#include "stepper.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "driver/ledc.h"
#include "freertos/queue.h"

#include "../include/main.h"
#include "esp_log.h"
// #include "driver/pcnt.h"
#include "driver/pulse_cnt.h" // TODO: migrar

#include "soc/gpio_sig_map.h"

// NEMA 17 1/32
#define FREQ_MIN  500//1500
#define FREQ_MAX  30000     // <--- VEL MAX

// // impresora 1/32
// #define FREQ_MIN  3500
// #define FREQ_MAX  7000

// // impresora 1/1
// #define FREQ_MIN  100
// #define FREQ_MAX  500

#define CPU_STEPPER     1

#define SPEED_MODE_TIMER    LEDC_LOW_SPEED_MODE
#define TIMER_MOT_L         LEDC_TIMER_0
#define TIMER_MOT_R         LEDC_TIMER_1

#define CHANNEL_MOT_L       LEDC_CHANNEL_0
#define CHANNEL_MOT_R       LEDC_CHANNEL_1

pcnt_unit_handle_t pcntUnitL = NULL;
pcnt_unit_handle_t pcntUnitR = NULL;

char *TAG = "PULSE CNT INT";

extern QueueHandle_t motorControlQueueHandler; 
static stepper_config_t configInit;
motors_measurements_t motorsMeasurements;

static void setVelMotors(int16_t speedL,int16_t speedR);
static void setEnableMotors(uint8_t enable);

motors_measurements_t getMeasMotors() { 
    return motorsMeasurements;
}

static void controlHandler(void *pvParameters) {

    output_motors_t newVel;
    while(true) {
        if (xQueueReceive(motorControlQueueHandler,&newVel,pdMS_TO_TICKS(1))) {
            if (newVel.enable) {
                setVelMotors(newVel.motorL,newVel.motorR);
            }
            else {
                setVelMotors(0,0);
            }
            setEnableMotors(newVel.enable);                         // TODO: esto se va a llamar continuamente
        }

        // ESP_ERROR_CHECK(pcnt_unit_get_count(pcntUnitL, &motorsMeasurements.absPosL));
        // ESP_ERROR_CHECK(pcnt_unit_get_count(pcntUnitR, &motorsMeasurements.absPosR));
        // ESP_LOGE("PCNT PRUEBAS","posL: %d,\tposR: %d",posL,posR);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool positionReachLimitsL(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    if (edata->watch_point_value == LOW_LIMIT_PCNT) {
        motorsMeasurements.absPosL -= 100;
    }
    else if (edata->watch_point_value == HIGH_LIMIT_PCNT) {
        motorsMeasurements.absPosL += 100;
    }
    return false; // TODO: revisar esto
}

static bool positionReachLimitsR(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    if (edata->watch_point_value == LOW_LIMIT_PCNT) {
        motorsMeasurements.absPosR -= 100;
    }
    else if (edata->watch_point_value == HIGH_LIMIT_PCNT) {
        motorsMeasurements.absPosR += 100;
    }
    return false; // TODO: revisar esto
}

static void initPositionSensor() {
    pcnt_unit_config_t unit_config = {
        .high_limit = HIGH_LIMIT_PCNT,
        .low_limit = LOW_LIMIT_PCNT,
    };

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcntUnitL));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcntUnitR));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcntUnitL, &filter_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcntUnitR, &filter_config));

    pcnt_chan_config_t configChannelL = {
        .edge_gpio_num = configInit.gpio_mot_l_step,
        .level_gpio_num = configInit.gpio_mot_l_dir,
    };
    pcnt_chan_config_t configChannelR = {
        .edge_gpio_num = configInit.gpio_mot_r_step,
        .level_gpio_num = configInit.gpio_mot_r_dir,
    };
    pcnt_channel_handle_t pcntChannelL = NULL;
    pcnt_channel_handle_t pcntChannelR = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcntUnitL, &configChannelL, &pcntChannelL));
    ESP_ERROR_CHECK(pcnt_new_channel(pcntUnitR, &configChannelR, &pcntChannelR));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcntChannelL, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcntChannelL, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcntChannelR, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcntChannelR, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcntUnitL, unit_config.low_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcntUnitL, unit_config.high_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcntUnitR, unit_config.low_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcntUnitR, unit_config.high_limit));

    pcnt_event_callbacks_t callbackReach = {
        .on_reach = positionReachLimitsL,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcntUnitL, &callbackReach, NULL));
    callbackReach.on_reach = positionReachLimitsR;
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcntUnitR, &callbackReach, NULL));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcntUnitL));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcntUnitR));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcntUnitL));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcntUnitR));
    ESP_ERROR_CHECK(pcnt_unit_start(pcntUnitL));
    ESP_ERROR_CHECK(pcnt_unit_start(pcntUnitR));

    esp_rom_gpio_connect_out_signal(configInit.gpio_mot_l_step, LEDC_LS_SIG_OUT0_IDX, false,false);
    esp_rom_gpio_connect_out_signal(configInit.gpio_mot_l_dir, SIG_GPIO_OUT_IDX, false,false);

    esp_rom_gpio_connect_out_signal(configInit.gpio_mot_r_step, LEDC_LS_SIG_OUT1_IDX, false,false);
    esp_rom_gpio_connect_out_signal(configInit.gpio_mot_r_dir, SIG_GPIO_OUT_IDX, false,false);
}

static void initPulseGenerator() {
    ledc_timer_config_t timerConfig = {
        .speed_mode = SPEED_MODE_TIMER,
        .timer_num = TIMER_MOT_L,
        .clk_cfg = LEDC_AUTO_CLK,
        .duty_resolution = LEDC_TIMER_4_BIT,
        .freq_hz = FREQ_MIN,
    };

    ledc_timer_config(&timerConfig);            // Timer config motor L

    timerConfig.timer_num = TIMER_MOT_R;
    ledc_timer_config(&timerConfig);            // Timer config motor R

    ledc_channel_config_t channelConfig = {
        .gpio_num = configInit.gpio_mot_l_step,
        .speed_mode = SPEED_MODE_TIMER,
        .channel = CHANNEL_MOT_L,
        .intr_type =  LEDC_INTR_DISABLE,
        .timer_sel = TIMER_MOT_L,
        .duty = 0,                              // 0 -> 0% de duty, 8 -> 50% de duty
        .hpoint = 0,
    };
    ledc_channel_config(&channelConfig);        // Channel config motor L

    channelConfig.gpio_num = configInit.gpio_mot_r_step;
    channelConfig.channel = CHANNEL_MOT_R;
    channelConfig.timer_sel = TIMER_MOT_R;

    ledc_channel_config(&channelConfig);        // Channel config motor R

    ledc_timer_pause(SPEED_MODE_TIMER,TIMER_MOT_L);
    ledc_timer_pause(SPEED_MODE_TIMER,TIMER_MOT_R);

    setEnableMotors(false);
    setVelMotors(0,0);
}

void motorsInit(stepper_config_t config) {

    configInit = config;
    /* seteo pines de salida de steps */
    gpio_config_t pinesMotor = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = ( 1 << config.gpio_mot_l_dir),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&pinesMotor);

    pinesMotor.pin_bit_mask = (1 << config.gpio_mot_r_dir);
    gpio_config(&pinesMotor);

    pinesMotor.pin_bit_mask = (1 << config.gpio_mot_l_step);
    gpio_config(&pinesMotor);

    pinesMotor.pin_bit_mask = (1 << config.gpio_mot_r_step);
    gpio_config(&pinesMotor);

    /* seteo pines de salida de enable*/
    pinesMotor.pin_bit_mask = (1 << config.gpio_mot_enable);
    pinesMotor.mode = GPIO_MODE_OUTPUT;
    gpio_config(&pinesMotor);

    pinesMotor.pin_bit_mask = (1 << config.gpio_mot_microstepper);
    gpio_config(&pinesMotor);

    initPulseGenerator();
    initPositionSensor();
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
}

void setVelMotors(int16_t speedL,int16_t speedR) {
    uint32_t timerFreqL=0,timerFreqR=0;

    if (!speedL) {
        ledc_set_duty(SPEED_MODE_TIMER,CHANNEL_MOT_L,0);
        ledc_update_duty(SPEED_MODE_TIMER,CHANNEL_MOT_L);
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
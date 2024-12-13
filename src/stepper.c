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
#include "driver/pcnt.h"
// #include "driver/pulse_cnt.h" // TODO: migrar

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

// pcnt_unit_handle_t pcnt_unit = NULL;

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

        int16_t posL = 0,posR = 0;
        pcnt_get_counter_value(PCNT_UNIT_0,&posL);
        pcnt_get_counter_value(PCNT_UNIT_1,&posR);

        motorsMeasurements.absPosL = posL;
        motorsMeasurements.absPosR = posR;

        // ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &posL)); // TODO: migrar
        // ESP_LOGE("PCNT PRUEBAS","posL: %d,\tposR: %d",posL,posR);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void initPositionSensor() {

    // TODO: migrar a pulse cnt
    // pcnt_unit_config_t unit_config = {
    //     .high_limit = 100,
    //     .low_limit = -100,
    // };

    // ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // ESP_LOGE(TAG, "set glitch filter");
    // pcnt_glitch_filter_config_t filter_config = {
    //     .max_glitch_ns = 1000,
    // };
    // ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    // ESP_LOGE(TAG, "install pcnt channels");
    // pcnt_chan_config_t chan_a_config = {
    //     .edge_gpio_num = configInit.gpio_mot_l_step,
    //     .level_gpio_num = configInit.gpio_mot_l_dir,
    // };
    // pcnt_channel_handle_t pcnt_chan_a = NULL;
    // ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    // ESP_LOGE(TAG, "set edge and level actions for pcnt channels");
    // ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    // ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // // ESP_LOGE(TAG, "add watch points and register callbacks");
    // // int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
    // // for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
    // //     ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    // // }
    // // pcnt_event_callbacks_t cbs = {
    // //     .on_reach = example_pcnt_on_reach,
    // // };
    // // QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    // // ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

    // ESP_LOGE(TAG, "enable pcnt unit");
    // ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    // ESP_LOGE(TAG, "clear pcnt unit");
    // ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    // ESP_LOGE(TAG, "start pcnt unit");
    // ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit))

    // pcnt_unit_config_t pcntConfig = {
    //     .low_limit = -100,
    //     .high_limit = 100,
    //     .flags.accum_count = 10,
    //     .intr_priority = 0
    // };
    // ESP_ERROR_CHECK(pcnt_new_unit(&pcntConfig, &pcnt_unit));

    // pcnt_chan_config_t pcntChannelConfig = {
    //     .edge_gpio_num = configInit.gpio_mot_l_step,
    //     .level_gpio_num = configInit.gpio_mot_l_dir,
        
    // };
    // pcnt_channel_handle_t pcnt_chan = NULL;
    // pcnt_new_channel(pcnt_unit, &pcntChannelConfig, &pcnt_chan);

    // ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // // watchpoints: 
    // // ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, EXAMPLE_PCNT_HIGH_LIMIT));

    // pcnt_unit_enable(pcnt_unit);
    // pcnt_unit_start(pcnt_unit);
    
    pcnt_config_t pcntConfig = {
        .pulse_gpio_num = configInit.gpio_mot_l_step,
        .ctrl_gpio_num = configInit.gpio_mot_l_dir,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_INC,         // Contar flancos positivos
        .neg_mode = PCNT_COUNT_DIS,         // Ignorar flancos negativos
        .lctrl_mode = PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        .hctrl_mode = PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
        .counter_h_lim = 32000,
        .counter_l_lim = -32000,
    };
    // Configuro pcnt para motor L
    pcnt_unit_config(&pcntConfig);

    // Configuro pcnt para motor R
    pcntConfig.pulse_gpio_num = configInit.gpio_mot_r_step,
    pcntConfig.ctrl_gpio_num = configInit.gpio_mot_r_dir,
    pcntConfig.unit = PCNT_UNIT_1,
    pcnt_unit_config(&pcntConfig);

    esp_rom_gpio_connect_out_signal(configInit.gpio_mot_l_step, LEDC_LS_SIG_OUT0_IDX, false,false);
    esp_rom_gpio_connect_out_signal(configInit.gpio_mot_l_dir, SIG_GPIO_OUT_IDX, false,false);

    esp_rom_gpio_connect_out_signal(configInit.gpio_mot_r_step, LEDC_LS_SIG_OUT1_IDX, false,false);
    esp_rom_gpio_connect_out_signal(configInit.gpio_mot_r_dir, SIG_GPIO_OUT_IDX, false,false);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
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
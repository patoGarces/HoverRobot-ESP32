#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#include "comms.h"
#include "storage_flash.h"
#include <string.h>

extern StreamBufferHandle_t xStreamBufferReceiver;
extern StreamBufferHandle_t xStreamBufferSender;

// queues de recepcion externa
extern QueueHandle_t newPidParamsQueueHandler;
extern QueueHandle_t receiveControlQueueHandler;
extern QueueHandle_t newCommandQueueHandler;

TaskHandle_t commsHandle;

static void communicationHandler(void * param);

void spp_wr_task_start_up(void){
    xTaskCreatePinnedToCore(communicationHandler, "communicationHandler", 4096, NULL, 10, &commsHandle,COMMS_HANDLER_CORE);
}

void spp_wr_task_shut_down(void) {
    vTaskDelete(commsHandle);
}

uint32_t getUint32( uint32_t index, char* payload){
    return (((uint32_t)payload[index+3]) << 24) + (((uint32_t)payload[index+2]) << 16) + (((uint32_t)payload[index+1]) << 8) + payload[index];
}

uint32_t getUint16( uint16_t index, char* payload){
    return (((uint32_t)payload[index+1]) << 8) + payload[index];
}

void communicationHandler(void * param) {
    char received_data[100];
    uint16_t contTimeout = 0;
    pid_settings_raw_t  newPidSettings;
    pid_params_t        pidParams;
    control_app_raw_t   newControlVal;
    command_app_raw_t   newCommand;
    
    while(true) {
        BaseType_t bytes_received = xStreamBufferReceive(xStreamBufferReceiver, received_data, sizeof(received_data), 0);//25);

        if (bytes_received > 1) {
            uint16_t headerPackage = getUint16(0,received_data);
            switch(headerPackage) {
                case HEADER_PACKAGE_SETTINGS: 
                    if (bytes_received == sizeof(newPidSettings)) {
                        memcpy(&newPidSettings,received_data,bytes_received);
 
                        pidParams.safetyLimits = newPidSettings.safetyLimits / PRECISION_DECIMALS_COMMS;
                        pidParams.centerAngle = newPidSettings.centerAngle / PRECISION_DECIMALS_COMMS;
                        pidParams.kp = newPidSettings.kp / PRECISION_DECIMALS_COMMS;
                        pidParams.ki = newPidSettings.ki / PRECISION_DECIMALS_COMMS;
                        pidParams.kd = newPidSettings.kd / PRECISION_DECIMALS_COMMS;
                        xQueueSend(newPidParamsQueueHandler,(void*)&pidParams,0);
                    }
                break;

                case HEADER_PACKAGE_CONTROL:
                    // if (bytes_received == sizeof(newControlVal)) {  
                    memcpy(&newControlVal,received_data,sizeof(newControlVal));//bytes_received);
                    contTimeout = 0;
                    // printf("NewControl recibido!\n");
                    xQueueSend(receiveControlQueueHandler,(void*)&newControlVal,0);
                         
                    // }
                    // else {
                    //     esp_log_buffer_hex("COMMS_HANDLER",(uint8_t *)(received_data),bytes_received);
                    //     printf("\n***** SIZEOFF NO COINCIDE: %d , %d*****\n",bytes_received,sizeof(newControlVal));
                    // }
                break;

                case HEADER_PACKAGE_COMMAND:
                    if (bytes_received == sizeof(newCommand)) { 
                        memcpy(&newCommand,received_data,bytes_received); 
                        xQueueSend(newCommandQueueHandler,(void*)&newCommand,0); 
                    }
                break;
                default:
                    printf("\n\nComando no reconocido: %x\n\n",headerPackage); 
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  
        contTimeout++;
        if (contTimeout > TIMEOUT_COMMS) {
            newControlVal.axisX = 0;
            newControlVal.axisY = 0;
            xQueueSend(receiveControlQueueHandler,(void*)&newControlVal,0);
        }
    }
    vTaskDelete(NULL);
}

void sendDynamicData(robot_dynamic_data_t dynamicData) {

    dynamicData.headerPackage = HEADER_PACKAGE_STATUS;

    if (xStreamBufferSend(xStreamBufferSender, &dynamicData, sizeof(dynamicData), 1) != sizeof(dynamicData)) {
        /* TODO: Manejar el caso en el que el buffer está lleno y no se pueden enviar datos */
        ESP_LOGI("COMMS", "BUFFER DE TRANSMISION OVERFLOW");
    }
}

/*
 * Convierto los parametros pid en float a u16 o i16 segun corresponda
*/
static pid_params_raw_t convertPidParamsToRaw(pid_params_t pidParams) {

    pid_params_raw_t pidRaw = {
        .kp = pidParams.kp * PRECISION_DECIMALS_COMMS,
        .ki = pidParams.ki * PRECISION_DECIMALS_COMMS,
        .kd = pidParams.kd * PRECISION_DECIMALS_COMMS,
    };
    return pidRaw;
}

void sendLocalConfig(robot_local_configs_t localConfig) {
    robot_local_configs_raw_t localConfigRaw;
    
    localConfigRaw.headerPackage = HEADER_PACKAGE_LOCAL_CONFIG;
    localConfigRaw.centerAngle = localConfig.centerAngle * PRECISION_DECIMALS_COMMS;
    localConfigRaw.safetyLimits = localConfig.safetyLimits * PRECISION_DECIMALS_COMMS;

    for(uint8_t i=0;i<CANT_PIDS;i++) {
        localConfigRaw.pid[i] = convertPidParamsToRaw(localConfig.pids[i]);
    }

    ESP_LOGI("SendLocalConfig","center: %d, safety: %d",localConfigRaw.centerAngle,localConfigRaw.safetyLimits);

    if (xStreamBufferSend(xStreamBufferSender, &localConfigRaw, sizeof(localConfigRaw), 1) != sizeof(localConfigRaw)) {
        /* TODO: Manejar el caso en el que el buffer está lleno y no se pueden enviar datos */
         ESP_LOGI("COMMS", "BUFFER DE TRANSMISION OVERFLOW");
    }
}
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

static const char *TAG = "COMMS HANDLER";

static void communicationHandler(void * param);

void comms_start_up(void){
    xTaskCreatePinnedToCore(communicationHandler, "communicationHandler", 4096, NULL, 10, &commsHandle,COMMS_HANDLER_CORE);
}

uint32_t getUint32( uint32_t index, char* payload){
    return (((uint32_t)payload[index+3]) << 24) + (((uint32_t)payload[index+2]) << 16) + (((uint32_t)payload[index+1]) << 8) + payload[index];
}

uint32_t getUint16( uint16_t index, char* payload){
    return (((uint32_t)payload[index+1]) << 8) + payload[index];
}

static void communicationHandler(void * param) {
    char received_data[100];
    uint16_t contTimeout = 0;
    pid_settings_app_raw_t      newPidSettingsRaw;
    pid_settings_comms_t        pidSettingsComms;
    control_app_raw_t           newControlVal;
    command_app_raw_t           newCommand;
    
    while(true) {
        BaseType_t bytes_received = xStreamBufferReceive(xStreamBufferReceiver, received_data, sizeof(received_data), 0);//25);

        if (bytes_received > 1) {
            uint16_t headerPackage = getUint16(0,received_data);
            switch(headerPackage) {
                case HEADER_PACKAGE_SETTINGS: 
                    if (bytes_received == sizeof(newPidSettingsRaw)) {
                        memcpy(&newPidSettingsRaw,received_data,bytes_received);
 
                        pidSettingsComms.indexPid = newPidSettingsRaw.indexPid;            // TODO: actualizar nuevos pid_floats_t
                        pidSettingsComms.safetyLimits = newPidSettingsRaw.safetyLimits / PRECISION_DECIMALS_COMMS;
                        pidSettingsComms.centerAngle = newPidSettingsRaw.centerAngle / PRECISION_DECIMALS_COMMS;
                        pidSettingsComms.kp = newPidSettingsRaw.kp / PRECISION_DECIMALS_COMMS;
                        pidSettingsComms.ki = newPidSettingsRaw.ki / PRECISION_DECIMALS_COMMS;
                        pidSettingsComms.kd = newPidSettingsRaw.kd / PRECISION_DECIMALS_COMMS;
                        xQueueSend(newPidParamsQueueHandler,(void*)&pidSettingsComms,0);
                    }
                break;

                case HEADER_PACKAGE_CONTROL:
                    // if (bytes_received == sizeof(newControlVal)) {  
                    memcpy(&newControlVal,received_data,sizeof(newControlVal));//bytes_received);
                    contTimeout = 0;
                    xQueueSend(receiveControlQueueHandler,(void*)&newControlVal,0);

                break;

                case HEADER_PACKAGE_COMMAND:
                    if (bytes_received == sizeof(newCommand)) { 
                        memcpy(&newCommand,received_data,bytes_received); 
                        xQueueSend(newCommandQueueHandler,(void*)&newCommand,0); 
                    }
                break;
                default:
                    ESP_LOGE(TAG, "\n\nComando no reconocido: %x\n\n",headerPackage); 
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
        ESP_LOGI("COMMS", "Overflow stream buffer dynamic data, is full?: %d, resetting...",xStreamBufferIsFull(xStreamBufferSender));
        xStreamBufferReset(xStreamBufferSender);
    }
}

void sendLocalConfig(robot_local_configs_t localConfig) {
    robot_local_configs_comms_t localConfigRaw;

    localConfigRaw.headerPackage = HEADER_PACKAGE_LOCAL_CONFIG;
    localConfigRaw.centerAngle = localConfig.centerAngle * PRECISION_DECIMALS_COMMS;
    localConfigRaw.safetyLimits = localConfig.safetyLimits * PRECISION_DECIMALS_COMMS;

    for(uint8_t i=0;i<CANT_PIDS;i++) {
        localConfigRaw.pid[i] = convertPidFloatsToRaw(localConfig.pids[i]);
    }

    ESP_LOGI("SendLocalConfig","center: %d, safety: %d",localConfigRaw.centerAngle,localConfigRaw.safetyLimits);

    if (xStreamBufferSend(xStreamBufferSender, &localConfigRaw, sizeof(localConfigRaw), 1) != sizeof(localConfigRaw)) {
        /* TODO: Manejar el caso en el que el buffer está lleno y no se pueden enviar datos */
         ESP_LOGI("COMMS", "BUFFER DE TRANSMISION OVERFLOW");
    }
}
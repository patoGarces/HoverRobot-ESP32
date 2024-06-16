#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#include "comms.h"
#include "storage_flash.h"
#include <string.h>

#define COMMS_CORE  0

extern StreamBufferHandle_t xStreamBufferReceiver;
extern StreamBufferHandle_t xStreamBufferSender;

extern QueueHandle_t queueNewPidParams;
extern QueueHandle_t queueReceiveControl;
extern QueueHandle_t queueNewCommand;

TaskHandle_t commsHandle;

static void communicationHandler(void * param);

void spp_wr_task_start_up(void){
    xTaskCreatePinnedToCore(communicationHandler, "communicationHandler", 4096, NULL, 5, &commsHandle,COMMS_CORE);
}
void spp_wr_task_shut_down(void){
    vTaskDelete(commsHandle);
}

uint32_t getUint32( uint32_t index, char* payload){
    return (((uint32_t)payload[index+3]) << 24) + (((uint32_t)payload[index+2]) << 16) + (((uint32_t)payload[index+1]) << 8) + payload[index];
}

uint32_t getUint16( uint16_t index, char* payload){
    return (((uint32_t)payload[index+1]) << 8) + payload[index];
}

void communicationHandler(void * param){
    char received_data[100];
    uint16_t contTimeout = 0;
    pid_settings_t  newPidSettings;
    pid_params_t    pidParams;
    control_app_t   newControlVal;
    command_app_t   newCommand;

    // queueReceiveControl = xQueueCreate(1, sizeof(control_app_t));
    
    while(true) {
        BaseType_t bytes_received = xStreamBufferReceive(xStreamBufferReceiver, received_data, sizeof(received_data), 0);

        if (bytes_received > 1) {
            // esp_log_buffer_hex("COMMS_HANDLER",(uint8_t *)(received_data),bytes_received);

            uint16_t header = getUint16(0,received_data);
            uint16_t headerPackage = getUint16(2,received_data);

            // printf("header: %x, headerPackage: %x\n",header,headerPackage);
            if (header == HEADER_COMMS) {
                switch(headerPackage) {
                    case HEADER_RX_KEY_SETTINGS: 
                        if (bytes_received == sizeof(newPidSettings)) {
                            memcpy(&newPidSettings,received_data,bytes_received);
                            uint16_t calculateChecksum = (newPidSettings.header ^ newPidSettings.header_key ^ newPidSettings.kp ^ newPidSettings.ki ^ newPidSettings.kd ^ newPidSettings.center_angle ^ newPidSettings.safety_limits);
                            if(newPidSettings.checksum == calculateChecksum){   
                                pidParams.safetyLimits = newPidSettings.safety_limits / 100.00;
                                pidParams.centerAngle = newPidSettings.center_angle / 100.00;
                                pidParams.kp = newPidSettings.kp / 100.00;
                                pidParams.ki = newPidSettings.ki / 100.00;
                                pidParams.kd = newPidSettings.kd / 100.00;

                                printf("\n######## Nuevo PID RECIBIDO RAAAY #######\n");
                                xQueueSend(queueNewPidParams,(void*)&pidParams,0);
                            }
                            else{
                                printf("ERROR CHECKSUM pidSettings: calc: %x receive: %x\n",calculateChecksum, newPidSettings.checksum);
                            }
                        }
                    break;

                    case HEADER_RX_KEY_CONTROL:
                        if (bytes_received == sizeof(newControlVal)) {  
                            memcpy(&newControlVal,received_data,bytes_received);
                            uint16_t calculateChecksum = (newControlVal.header ^ newControlVal.header_key ^ newControlVal.axis_x ^ newControlVal.axis_y);
                            if(newControlVal.checksum == calculateChecksum) {   
                                contTimeout = 0;
                                printf("NewControl recibido!\n");
                                xQueueSend(queueReceiveControl,(void*)&newControlVal,0);
                            }
                            else{
                                printf("ERROR CHECKSUM newControlVal: calc: %x receive: %x\n",calculateChecksum, newControlVal.checksum);
                            }      
                        }
                        else {
                            esp_log_buffer_hex("COMMS_HANDLER",(uint8_t *)(received_data),bytes_received);
                            printf("\n***** SIZEOFF NO COINCIDE: %d , %d*****\n",bytes_received,sizeof(newControlVal));
                        }
                    break;

                    case HEADER_TX_KEY_COMMAND:
                        if (bytes_received == sizeof(newCommand)) { 
                            memcpy(&newCommand,received_data,bytes_received); 
                            if(newCommand.checksum == (newCommand.header ^ newCommand.header_key ^ newCommand.command)) {   
                                xQueueSend(queueNewCommand,(void*)&newCommand,0);
                            }
                            else{
                                printf("ERROR CHECKSUM newCommand\n");
                            }      
                        }
                    break;
                    default:
                        printf("\n\nComando no reconocido\n\n"); 
                    break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  
        contTimeout++;
        if (contTimeout > TIMEOUT_COMMS) {
            newControlVal.axis_x = 0;
            newControlVal.axis_y = 0;
            xQueueSend(queueReceiveControl,(void*)&newControlVal,0);
        }
    }
    vTaskDelete(NULL);
}

void sendDynamicData(robot_dynamic_data_t dynamicData) {

    dynamicData.header = HEADER_TX_KEY_STATUS;

    dynamicData.checksum = dynamicData.header ^
                dynamicData.speedR ^
                dynamicData.speedL ^
                dynamicData.pitch ^
                dynamicData.roll ^
                dynamicData.yaw ^
                dynamicData.setPoint ^
                dynamicData.centerAngle ^
                dynamicData.statusCode;

    if (xStreamBufferSend(xStreamBufferSender, &dynamicData, sizeof(dynamicData), 1) != sizeof(dynamicData)) {
        /* TODO: Manejar el caso en el que el buffer está lleno y no se pueden enviar datos */
         ESP_LOGI("COMMS", "BUFFER DE TRANSMISION OVERFLOW");
    }

    // char SendAngleChar[50];
    // // sprintf(SendAngleChar,">angle:%f\n>outputMotor:%f\n",status.actualAngle,status.speedL/100.0);
    // sprintf(SendAngleChar,">angle:%f\n",status.actualAngle);
    // printf(SendAngleChar); 
    // if (xStreamBufferSend(xStreamBufferSender, &SendAngleChar, sizeof(SendAngleChar), 1) != pdPASS) {
    //     /* TODO: Manejar el caso en el que el buffer está lleno y no se pueden enviar datos */
    // }
}

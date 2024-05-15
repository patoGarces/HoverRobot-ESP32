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

static void communicationHandler(void * param);

void spp_wr_task_start_up(void){
    xTaskCreatePinnedToCore(communicationHandler, "communicationHandler", 4096, NULL, 5, NULL,COMMS_CORE);
}
void spp_wr_task_shut_down(void){
    vTaskDelete(NULL);
}

uint32_t getUint32( uint32_t index, char* payload){
    return (((uint32_t)payload[index+3]) << 24) + (((uint32_t)payload[index+2]) << 16) + (((uint32_t)payload[index+1]) << 8) + payload[index];
}

void communicationHandler(void * param){
    char received_data[100];

    pid_settings_t  newPidSettings;
    pid_params_t    pidParams;
    control_app_t   newControlVal;
    command_app_t   newCommand;
    
    while(1) {
        BaseType_t bytes_received = xStreamBufferReceive(xStreamBufferReceiver, received_data, sizeof(received_data), 0);

        if (bytes_received > 1) {

            uint32_t header = getUint32(0,received_data);
            uint32_t headerPackage = getUint32(4,received_data);
            if (header == HEADER_COMMS) {
                switch(headerPackage) {
                    case HEADER_RX_KEY_SETTINGS: 
                        if (bytes_received == sizeof(newPidSettings)) {
                            memcpy(&newPidSettings,received_data,bytes_received);
                            if(newPidSettings.checksum == (newPidSettings.header ^ newPidSettings.header_key ^ newPidSettings.kp ^ newPidSettings.ki ^ newPidSettings.kd ^ newPidSettings.center_angle ^ newPidSettings.safety_limits)){   
                                pidParams.safetyLimits = newPidSettings.safety_limits / 100.00;
                                pidParams.centerAngle = newPidSettings.center_angle / 100.00;
                                pidParams.kp = newPidSettings.kp / 100.00;
                                pidParams.ki = newPidSettings.ki / 100.00;
                                pidParams.kd = newPidSettings.kd / 100.00;
                                xQueueSend(queueNewPidParams,(void*)&pidParams,0);
                            }
                            else{
                                printf("ERROR CHECKSUM pidSettings\n");
                            }
                        }
                    break;

                    case HEADER_RX_KEY_CONTROL:
                        if (bytes_received == sizeof(newControlVal)) {  
                            memcpy(&newControlVal,received_data,bytes_received);
                            if(newControlVal.checksum == (newControlVal.header ^ newControlVal.header_key ^ newControlVal.axis_x ^ newControlVal.axis_y)) {   
                                xQueueSend(queueReceiveControl,(void*)&newControlVal,0);
                            }
                            else{
                                printf("ERROR CHECKSUM newControlVal\n");
                            }      
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
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  
    }
    spp_wr_task_shut_down();
}

void sendStatus(status_robot_t status){

    status.checksum =   status.header ^
                        status.batVoltage ^
                        status.batPercent ^
                        status.batTemp ^
                        status.tempImu ^
                        status.tempEscs ^
                        status.speedR ^
                        status.speedL ^
                        (uint16_t)status.pitch ^
                        (uint16_t)status.roll ^
                        (uint16_t)status.yaw ^
                        (uint16_t)status.pid.kp ^
                        (uint16_t)status.pid.ki ^
                        (uint16_t)status.pid.kd ^
                        (uint16_t)status.pid.safetyLimits ^
                        (uint16_t)status.pid.centerAngle ^
                        (uint16_t)status.setPoint ^
                        status.ordenCode ^
                        status.statusCode;


    if (xStreamBufferSend(xStreamBufferSender, &status, sizeof(status), 1) != sizeof(status)) {
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

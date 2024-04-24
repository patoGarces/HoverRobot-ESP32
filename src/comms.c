
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"
#include "comms.h"
#include "storage_flash.h"
#include <string.h>


#define COMMS_CORE  0

// QueueHandle_t queueReceiveSettings;

extern StreamBufferHandle_t xStreamBufferReceiver;
extern StreamBufferHandle_t xStreamBufferSender;

extern QueueHandle_t queueNewPidParams;
extern QueueHandle_t queueReceiveControl;

void communicationHandler(void * param);

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
    
    while(1) {
        BaseType_t bytes_received = xStreamBufferReceive(xStreamBufferReceiver, received_data, sizeof(received_data), 0);

        if (bytes_received > 1){
            // esp_log_buffer_hex("xStreamBufferReceive", &received_data, bytes_received);

            uint32_t header = getUint32(0,received_data);
            uint32_t headerPackage = getUint32(4,received_data);
            if( header == HEADER_COMMS && headerPackage == HEADER_RX_KEY_SETTINGS && bytes_received == sizeof(newPidSettings)){

                memcpy(&newPidSettings,received_data,bytes_received);

                if(newPidSettings.header == HEADER_COMMS && (newPidSettings.checksum == (newPidSettings.header ^ newPidSettings.header_key ^ newPidSettings.kp ^ newPidSettings.ki ^ newPidSettings.kd ^ newPidSettings.center_angle ^ newPidSettings.safety_limits))){   
                    pidParams.safetyLimits = newPidSettings.safety_limits / 10.00;
                    pidParams.centerAngle = newPidSettings.center_angle / 10.00;
                    pidParams.kp = newPidSettings.kp / 10.00;
                    pidParams.ki = newPidSettings.ki / 10.00;
                    pidParams.kd = newPidSettings.kd / 10.00;
                    xQueueSend(queueNewPidParams,(void*)&pidParams,0);
                }
                else{
                    printf("ERROR CHECKSUM pidSettings\n");
                }
            }
            else if( header == HEADER_COMMS && headerPackage == HEADER_RX_KEY_CONTROL && bytes_received == sizeof(newControlVal)){
                memcpy(&newControlVal,received_data,bytes_received);
                xQueueSend(queueReceiveControl,(void*)&newControlVal,0);
            }

        }
        vTaskDelay(pdMS_TO_TICKS(10));  
    }

    spp_wr_task_shut_down();
}

void sendStatus(status_robot_t status){
    // xQueueSend(queueSend,( void * ) &status, 0);

    if (xStreamBufferSend(xStreamBufferSender, &status, sizeof(status), 1) != pdPASS) {
        /* TODO: Manejar el caso en el que el buffer está lleno y no se pueden enviar datos */
    }

    
    // char SendAngleChar[50];

    // // sprintf(SendAngleChar,">angle:%f\n>outputMotor:%f\n",status.actualAngle,status.speedL/100.0);
    // sprintf(SendAngleChar,">angle:%f\n",status.actualAngle);

    // printf(SendAngleChar);
           
    // if (xStreamBufferSend(xStreamBufferSender, &SendAngleChar, sizeof(SendAngleChar), 1) != pdPASS) {
    //     /* TODO: Manejar el caso en el que el buffer está lleno y no se pueden enviar datos */
    // }
}

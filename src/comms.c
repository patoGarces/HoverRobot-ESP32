
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "comms.h"
#include "storage_flash.h"

#define COMMS_CORE  0

QueueHandle_t queueReceiveSettings;
// QueueHandle_t queueReceiveControl;
extern QueueHandle_t queueSend;
extern QueueHandle_t queueNewPidParams;


void spp_wr_task_start_up(spp_wr_task_cb_t p_cback, int fd)
{
    xTaskCreatePinnedToCore(p_cback, "write_read", 2048, (void *)fd, 5, NULL,COMMS_CORE);
}
void spp_wr_task_shut_down(void)
{
    vTaskDelete(NULL);
}

void spp_read_handle(void * param)
{
    queueReceiveSettings = xQueueCreate(1, sizeof(pid_settings_t));
    // queueReceiveControl = xQueueCreate(1, sizeof(control_app_t));

    pid_settings_t newPidSettings;
    // control_app_t newControlVal;
    pid_params_t pidParams;
    
    while (1){
        if( xQueueReceive(queueReceiveSettings,
                         &newPidSettings,
                         ( TickType_t ) 1 ) == pdPASS ){
            
            // esp_log_buffer_hex("ENQUEUE RECEIVE:", &pidSettings, sizeof(pidSettings));

            if(newPidSettings.header == HEADER_COMMS && (newPidSettings.checksum == (newPidSettings.header ^ newPidSettings.header_key ^ newPidSettings.kp ^ newPidSettings.ki ^ newPidSettings.kd^ newPidSettings.centerAngle))){
                printf("KP = %ld, KI = %ld, KD = %ld, centerAngle: %ld, checksum: %ld\n", newPidSettings.kp,newPidSettings.ki,newPidSettings.kd,newPidSettings.centerAngle,newPidSettings.checksum);
                
                // pidParams.centerAngle = (float)newPidSettings.centerAngle/100;
                pidParams.centerAngle = (float)newPidSettings.centerAngle;
                pidParams.kp = (float)newPidSettings.kp/100;
                pidParams.ki = (float)newPidSettings.ki/100;
                pidParams.kd = (float)newPidSettings.kd/100;
                xQueueSend(queueNewPidParams,(void*)&pidParams,0);
            }
        }
        // if( xQueueReceive(queueReceiveControl,
        //                  &newControlVal,
        //                  ( TickType_t ) 1 ) == pdPASS ){
        //     printf("CONTROL RECIBIDO: X: %ld, Y: %ld\n",newControlVal.axis_x,newControlVal.axis_y);
        // }   
    }

    spp_wr_task_shut_down();
}

void sendStatus(status_robot_t status){
    xQueueSend(queueSend,( void * ) &status, 0);
}
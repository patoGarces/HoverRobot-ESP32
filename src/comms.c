
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "comms.h"
#include "storage_flash.h"

#define COMMS_CORE  0

QueueHandle_t queueReceive;
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
    queueReceive = xQueueCreate(1, sizeof(pid_settings_t));

    pid_settings_t newPidSettings;
    pid_params_t pidParams;
    
    do {
        if( xQueueReceive(queueReceive,
                         &newPidSettings,
                         ( TickType_t ) 100 ) == pdPASS ){
            
            // esp_log_buffer_hex("ENQUEUE RECEIVE:", &pidSettings, sizeof(pidSettings));

            if(newPidSettings.header == HEADER_COMMS && (newPidSettings.checksum == (newPidSettings.header ^ newPidSettings.kp ^ newPidSettings.ki ^ newPidSettings.kd))){
                printf("KP = %ld, KI = %ld, KD = %ld, checksum: %ld\n", newPidSettings.kp,newPidSettings.ki,newPidSettings.kd,newPidSettings.checksum);
                
                // pidParams.centerAngle = (float)newPidSettings.centerAngle/100;
                pidParams.centerAngle = -0.15;
                pidParams.kp = (float)newPidSettings.kp/100;
                pidParams.ki = (float)newPidSettings.ki/100;
                pidParams.kd = (float)newPidSettings.kd/100;
                xQueueSend(queueNewPidParams,(void*)&pidParams,0);
            }
        }
    } while (1);

    spp_wr_task_shut_down();
}

void sendStatus(status_robot_t status){
    xQueueSend(queueSend,( void * ) &status, 0);
}
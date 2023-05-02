
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "comms.h"

#define COMMS_CORE  0

QueueHandle_t queueReceive;
QueueHandle_t queueSend;


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
    queueReceive = xQueueCreate(1, sizeof(pidSettings_t));
    queueSend = xQueueCreate(1, sizeof(status_robot_t));

    pidSettings_t pidSettings;
 
    pidSettings.header = 0;
    pidSettings.kp = 0;
    pidSettings.ki = 0;
    pidSettings.kd = 0;

    do {

        if( xQueueReceive(queueReceive,
                         &pidSettings,
                         ( TickType_t ) 100 ) == pdPASS ){
            
            // esp_log_buffer_hex("ENQUEUE RECEIVE:", &pidSettings, sizeof(pidSettings));
            // if(pidSettings.header == 0xABC0){
            //     printf("HEader detectado! \n");
            // }

            if(pidSettings.checksum == (pidSettings.header ^ pidSettings.kp ^ pidSettings.ki ^ pidSettings.kd)){
                printf("KP = %ld, KI = %ld, KD = %ld, checksum: %ld\n", pidSettings.kp,pidSettings.ki,pidSettings.kd,pidSettings.checksum);
            }

        }
    } while (1);

    spp_wr_task_shut_down();
}

void sendStatus(status_robot_t status){
    xQueueSend(queueSend,( void * ) &status, 0);
}
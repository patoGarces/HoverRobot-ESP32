
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "comms.h"

#define HEADER_COMMS 0xABC0

QueueHandle_t queueReceive;
QueueHandle_t queueSend;


void spp_wr_task_start_up(spp_wr_task_cb_t p_cback, int fd)
{
    xTaskCreate(p_cback, "write_read", 2048, (void *)fd, 5, NULL);
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
                
                status_robot_t status={
                    .header = HEADER_COMMS,
                    .bat_voltage = 11,
                    .bat_percent = 51,
                    .batTemp = 255,
                    .temp_uc_control = 115,
                    .temp_uc_main = 266,
                    .speedR = 0,
                    .speedL = 0,
                    .pitch = 10664,
                    .roll = 1198456,
                    .yaw = -65564,
                    .centerAngle = 0,
                    .P = pidSettings.kp,
                    .I = pidSettings.ki,
                    .D = pidSettings.kd,   
                    .orden_code = 15,
                    .error_code = 1,//ERROR_CODE_INIT,
                    .checksum = 0
                };
                sendDato(status);
            }

        }
    } while (1);

    spp_wr_task_shut_down();
}

void sendDato(status_robot_t status){
    printf("agrego dato a enviar a la cola\n");
    xQueueSend(queueSend,( void * ) &status, 0);
}

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "comms.h"



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
            }

        }
    } while (1);

    spp_wr_task_shut_down();
}

void sendDato(status_robot_t status){
    printf("agrego dato a enviar a la cola\n");
    xQueueSend(queueSend,( void * ) &status, 0);
}

void btSendData(float x,float y, uint16_t motores){
    // char spp_data[256];
    // sprintf(spp_data, "X: %f Y: %f, Motores: %d\n", x,y,motores);
    // esp_spp_write(handleSpp, strlen(spp_data), (uint8_t *)spp_data);
}

void btSendAngle(float ejeX,float ejeY,float ejeZ){
    // char spp_data[256];
    // sprintf(spp_data,"Angle X: %f\fAngle Y: %f\fAngle Z: %f\f\n", ejeX, ejeY, ejeZ);
    // esp_spp_write(handleSpp, strlen(spp_data), (uint8_t *)spp_data);
}

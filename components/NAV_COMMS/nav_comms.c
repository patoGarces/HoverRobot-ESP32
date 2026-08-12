#include "include/nav_comms.h"
#include "string.h"
#include "esp_log.h"

config_init_nav_t navConfigInit;
static QueueHandle_t spp_uart_queue;
static const char *TAG = "NAV_COMMS";

static void controlHandler(void *pvParameters) {
    uart_event_t event;
    uint8_t data[256];
    char received_data[100];

    while(true) {
        BaseType_t bytesReceived = xStreamBufferReceive(navConfigInit.xStreamBufferSend, received_data, sizeof(received_data), 0);//25);
        
        if (bytesReceived > 0) {
            uart_write_bytes(navConfigInit.numUart,received_data, bytesReceived);
            // ESP_LOGI(TAG, "STREAM BUFFER SEND: %d bytes", bytesReceived);
        }

        if (xQueueReceive(spp_uart_queue, (void * )&event, 0)) {
            
            switch(event.type) {
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(navConfigInit.numUart);
                    xQueueReset(spp_uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider increasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(navConfigInit.numUart);
                    xQueueReset(spp_uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    // ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                
                // case UART_DATA:
                //     size_t sizeBuffer;
                //     uart_get_buffered_data_len(navConfigInit.numUart, &sizeBuffer);
                //     uart_read_bytes(navConfigInit.numUart, data, sizeBuffer,0);

                //     if (sizeBuffer > 0) {
                //         ESP_LOGI(TAG, "STREAM BUFFER RECEIVED: %d bytes", sizeBuffer);

                //         if (xStreamBufferSend(navConfigInit.xStreamBufferRecv, data, sizeBuffer, 1) != sizeBuffer) {
                //             /* TODO: Manejar el caso en el que el buffer está lleno y no se pueden enviar datos */
                //             ESP_LOGI(TAG, "Overflow stream buffer dynamic data, is full?: %d, resetting...", xStreamBufferIsFull(navConfigInit.xStreamBufferRecv));
                //             xStreamBufferReset(navConfigInit.xStreamBufferRecv);
                //         }
                //     }
                //     break;
                case UART_DATA: {
                    int len = uart_read_bytes(navConfigInit.numUart, data, sizeof(data), pdMS_TO_TICKS(10));
                    if (len > 0) {
                        // ESP_LOGI(TAG, "STREAM BUFFER RECEIVED: %d bytes", len);

                        if (xStreamBufferSend(navConfigInit.xStreamBufferRecv, data, len, 1) != len) {
                            ESP_LOGW(TAG, "Overflow stream buffer");
                        }
                    }
                    break;
                }
                default:
                    if (event.type != UART_DATA) {
                        ESP_LOGI(TAG, "unhandled event type: %d", event.type);
                    }
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void navComms(config_init_nav_t *config) {
    navConfigInit = *config;
    /*configuro periferico*/
    uart_config_t uartConfig={
        .baud_rate = config->baudrate,
        .data_bits = UART_DATA_8_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .parity = UART_PARITY_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .stop_bits = UART_STOP_BITS_1, 
        .source_clk = UART_SCLK_APB,
    };

    // buffer size of tx y rx
    const int uart_buffer_size = (1024 * 4);
    ESP_ERROR_CHECK(uart_driver_install(config->numUart, uart_buffer_size, uart_buffer_size, 10, &spp_uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(config->numUart,&uartConfig));
    /* configure pinout uart*/
    ESP_ERROR_CHECK(uart_set_pin(config->numUart,config->txPin,config->rxPin,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));

    xTaskCreatePinnedToCore(controlHandler, "NAV handler task", 4096, NULL, configMAX_PRIORITIES - 2, NULL, config->core);
    ESP_LOGI(TAG,"initialized");

    uint8_t clientSerialConnected = 1;      // TODO: mock para simular el estado conectado
    if (xQueueOverwrite(config->connectionQueueHandler, &clientSerialConnected) != pdPASS) {
        ESP_LOGE(TAG, "Error al enviar el nuevo estado de connection");
    }
}

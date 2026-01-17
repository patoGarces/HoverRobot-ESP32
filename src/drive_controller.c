#include "drive_controller.h"
#include "esp_log.h"
#include "main.h"

#ifdef HARDWARE_PROTOTYPE
    #include "IMC_CONTROL.h"
#else
    #include "CAN_MCB.h"
#endif

static const char* TAG = "drive_controller";

static QueueHandle_t motorControllerQueue;
static QueueHandle_t dataControllerQueue;

static QueueHandle_t motorControlBackendQueue;
static QueueHandle_t dataReceiveBackendQueue;

static float pos2mts(int32_t steps) {
    return (steps/STEPS_PER_REV) * DIST_PER_REV;
}

float rpm2mps(int16_t rpm) {
    return (rpm * DIST_PER_REV) / 60.00;
}

float mps2rpm(float mps) {
    return (mps * 60.00) * DIST_PER_REV;
}

void driveControllerHandlerTask(void *pvParameters) {

    TickType_t lastWakeTime = xTaskGetTickCount();
    drive_controller_data_t driveControllerData;
    drive_controller_motor_control_t newMotorControl;

    #ifdef HARDWARE_PROTOTYPE
        imc_data_received_t receivedDataFromImc;
    #else
        mcb_data_received_t receivedDataFromMcb;
    #endif

    while(true) {
        #ifdef HARDWARE_PROTOTYPE
            if (xQueueReceive(dataReceiveBackendQueue, &receivedDataFromImc, 0)) {
                driveControllerData = (drive_controller_data_t) {
                    .isCharging = false,
                    .boardTemp = 0.0f,
                    .batVoltage = 0.0f,
                    .speedMeasRms = receivedDataFromImc.speedMotR,
                    .speedMeasLms = receivedDataFromImc.speedMotL,
                    .currentR = 0.0f,
                    .currentL = 0.0f,
                    .posInMetersR = pos2mts(receivedDataFromImc.absPosR),
                    .posInMetersL = pos2mts(receivedDataFromImc.absPosL),
                    .statusCode = NO_ERROR_MCB
                };

                xQueueSend(dataControllerQueue, &driveControllerData, 0);
            }
        #else
            if (xQueueReceive(dataReceiveBackendQueue, &receivedDataFromMcb, 0)) {

                driveControllerData = (drive_controller_data_t) {
                    .isCharging = receivedDataFromMcb.isCharging,
                    .boardTemp = receivedDataFromMcb.boardTemp / 10.00,
                    .batVoltage = receivedDataFromMcb.batVoltage ,
                    .speedMeasRms = receivedDataFromMcb.speedR_meas,
                    .speedMeasLms = receivedDataFromMcb.speedL_meas,
                    .currentR = receivedDataFromMcb.currentR,
                    .currentL = receivedDataFromMcb.currentL,
                    .posInMetersR = pos2mts(receivedDataFromMcb.posR),
                    .posInMetersL = pos2mts(receivedDataFromMcb.posL * -1),
                    .statusCode = receivedDataFromMcb.statusCode
                };

                xQueueSend(dataControllerQueue, &driveControllerData, 0);

            //     if (!statusRobot.isMcbConnected) {
            //         statusRobot.isMcbConnected = true;
            //         if (statusRobot.statusCode == STATUS_ROBOT_ERROR_MCB_CONNECTION) {
            //             setStatusRobot(STATUS_ROBOT_ARMED);
            //         }
            //     }

            //     contMcbTimeout++;
            //     if(contMcbTimeout > maxMcbTicksTimeout) {
            //         statusRobot.isMcbConnected = false;
            //         setStatusRobot(STATUS_ROBOT_ERROR_MCB_CONNECTION);
            //     }
            }
        #endif

        if (xQueueReceive(motorControllerQueue, &newMotorControl, 0)) {
            xQueueSend(motorControlBackendQueue, &newMotorControl, 0);
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
    }
}

void driveControllerInit(QueueHandle_t driveMotorControlQueue, QueueHandle_t receivedDataQueue) {


    dataControllerQueue = receivedDataQueue;
    motorControllerQueue = driveMotorControlQueue;

    #ifdef HARDWARE_PROTOTYPE
        dataReceiveBackendQueue = xQueueCreate(1, sizeof(imc_data_received_t));
        motorControlBackendQueue = xQueueCreate(1, sizeof(imc_motor_control_t)) ;

        config_imc_init_t configImc = {
            .gpio_mot_l_step = GPIO_MOT_L_STEP,
            .gpio_mot_l_dir = GPIO_MOT_L_DIR,
            .gpio_mot_r_step = GPIO_MOT_R_STEP,
            .gpio_mot_r_dir = GPIO_MOT_R_DIR,
            .gpio_mot_enable = GPIO_MOT_ENABLE,
            .gpio_mot_microstepper = GPIO_MOT_MICRO_STEP,
            .queueSendControl = motorControlBackendQueue,
            .queueReceiveData = dataReceiveBackendQueue
        };
        imcInit(configImc);
        setMicroSteps(true);
    #else
        dataReceiveBackendQueue = xQueueCreate(1, sizeof(mcb_data_received_t));
        motorControlBackendQueue = xQueueCreate(1, sizeof(mcb_motor_control_t));

        config_init_mcb_t configMcb = {
            .numUart = UART_PORT_CAN,
            .txPin = GPIO_CAN_TX,
            .rxPin = GPIO_CAN_RX,
            .queueSendControl = motorControlBackendQueue,
            .queueReceiveData = dataReceiveBackendQueue,
            .core = IMU_HANDLER_CORE
        };
        mcbInit(&configMcb);
    #endif

    xTaskCreatePinnedToCore(driveControllerHandlerTask, "drive controller", 4096, NULL, configMAX_PRIORITIES - 2, NULL, IMU_HANDLER_CORE);
}
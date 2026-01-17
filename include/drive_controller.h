#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define TIMEOUT_MCB_MS      500.0

#ifdef HARDWARE_PROTOTYPE

    #define STEPS_PER_REV       6400.00                 // 200 steps * 1/32 microsteps = 6400 pulsos por vuelta
    #define DIST_PER_REV        0.326725635973          // diam 0.104m * pi = 0,326725635973 mts

    #define WHEEL_BASE          0.105                   // distancia entre ruedas en metros
    #define MAX_VELOCITY_CONTROL_IN_MPS    0.5             // Velocidad maxima para control en m/s

    #define DIRECTION_L_MOTOR  1
    #define DIRECTION_R_MOTOR  1
#else
    #define STEPS_PER_REV       90.00                   // 90 steps por vuelta
    #define DIST_PER_REV        0.5310707511            // diam 17cm * pi = 53.10707 cms = 0.5310707511 mts

    #define WHEEL_BASE          0.32                    // distancia entre ruedas en metros

    #define MAX_VELOCITY_CONTROL_IN_MPS    1.00         // Velocidad maxima para control en m/s

    #define INVERT_HALL_SIDE        // Invierte el sensor R con el L(depende de la ubicacion fisica de la MCB)

    #ifdef HARDWARE_SPLITBOARD
        #define DIRECTION_L_MOTOR  1
        #define DIRECTION_R_MOTOR  -1

    #elif defined(HARDWARE_MAINBOARD)

        #define DIRECTION_L_MOTOR  1
        #define DIRECTION_R_MOTOR  1
    #endif
#endif

typedef enum {
    NO_ERROR_MCB,               // TODO: cambiar los nombres para no hacer alusion a MCB
    ERROR_MCB_BATTERY,
    ERROR_MCB_TEMP,
    ERROR_MCB_HALL_L,
    ERROR_MCB_HALL_R,
    ERROR_MCB_INACTIVITY,
} drive_controller_status_code_t;

typedef struct {
    int16_t motorR;
    int16_t motorL;
    uint8_t enable;
} drive_controller_motor_control_t;

typedef struct {
    bool    isCharging;                 // Indicador si esta cargando la bateria
    float   boardTemp;                  // Temperatura de la placa de control en C
    float   batVoltage;                 // Tension de la bateria en V
    float   speedMeasRms;               // Velocidad rueda derecha en m/s
    float   speedMeasLms;               // Velocidad rueda izquierda en m/s
    float   currentR;                   // Corriente motor rueda derecha en A
    float   currentL;                   // Corriente motor rueda izquierda en A
    float   posInMetersR;               // Posicion rueda derecha en metros
    float   posInMetersL;               // Posicion rueda izquierda en metros
    drive_controller_status_code_t statusCode;
} drive_controller_data_t;


void driveControllerInit(QueueHandle_t driveMotorControlQueue, QueueHandle_t receivedDataQueue);
float rpm2mps(int16_t rpm);
float mps2rpm(float mps);
#include "PID.h"

#include "stdio.h"

pid_control_t pidControl[CANT_PIDS];

float normalize(float value);

esp_err_t pidInit(pid_init_t initParams[CANT_PIDS]) {
    for(uint8_t i = 0;i<CANT_PIDS;i++) { 
        if (initParams[i].sampleTimeInMs > 1000.00 || initParams[i].sampleTimeInMs < 1.00) {
            return ESP_FAIL;
        }
        pidControl[i].sampleTimeInSec = (float)initParams[i].sampleTimeInMs / 1000;
        pidControl[i].enablePID = false;
        pidSetConstants(i,initParams[i].kp, initParams[i].ki, initParams[i].kd);
        pidSetSetPoint(i,initParams[i].initSetPoint);
    }
    return ESP_OK;
}

void pidSetEnable(uint8_t numPid) {
    pidControl[numPid].enablePID = true;
}

void pidSetDisable(uint8_t numPid) {
    pidControl[numPid].enablePID = false;
    pidControl[numPid].iTerm = 0.00;
    pidControl[numPid].lastInput = 0.00;
}

bool pidGetEnable(uint8_t numPid) {
    return pidControl[numPid].enablePID;
}

float normalize(float value) {
    return (value / 100.00);
}

float cutNormalizeLimits(float normalizeInput) {
    if (normalizeInput > 1.00) {  // recorto el termino integral maximo
        return 1.00;
    } else if (normalizeInput < -1.00) {  // recorto el termino intgral minimo
        return -1.00;
    }
    return normalizeInput;
}

/*
 * Esta funcion debe ser llamada cada un periodo fijo definido en pidControl1.sampleTime
 * @param input entra en rango [-100;100]
 * @return resultado del PID normalizado [-1.00;1.00]
 */
float pidCalculate(uint8_t numPid,float input) {
        if (!pidControl[numPid].enablePID) {
            return 0;
        }
        
        float normalizeInput = normalize(input);
        float error = pidControl[numPid].setPoint - normalizeInput;  													  // Error: diferencia entre el valor seteado y el de entrada

        pidControl[numPid].iTerm += pidControl[numPid].sampleTimeInSec * pidControl[numPid].ki * error;                                // calculo I: acumulo error multiplicado por ki contemplando el tiempo transcurrido desd el anterior
        float dTerm = ((normalizeInput - pidControl[numPid].lastInput) * pidControl[numPid].kd) / pidControl[numPid].sampleTimeInSec;  // Calculo D: resto la entrada anterior a la actual

        pidControl[numPid].iTerm = cutNormalizeLimits(pidControl[numPid].iTerm);
        dTerm = cutNormalizeLimits(dTerm);

        /*Suma de errores*/
        float output = (pidControl[numPid].kp * error) + pidControl[numPid].iTerm - dTerm;  													// opero con los 3 parametros para obtener salida, el D se resta para evitar la kick derivate

        pidControl[numPid].lastInput = normalizeInput;
        return cutNormalizeLimits(output);
}

/*
 * Funcion para asignar un nuevo setPoint
 */
void pidSetSetPoint(uint8_t numPid,float value) {
    pidControl[numPid].setPoint = normalize(value);
}

/*
 * Funcion para obtener el setPoint actual
 */
float pidGetSetPoint(uint8_t numPid) {
    return pidControl[numPid].setPoint * 100.00;
}

/*
 * 	Funcion para cargar los parametros al filtro PID
 */
void pidSetConstants(uint8_t numPid,float KP, float KI, float KD) {
    pidControl[numPid].kp = KP;
    pidControl[numPid].ki = KI;
    pidControl[numPid].kd = KD;
    pidControl[numPid].iTerm = 0.00;
    pidControl[numPid].lastInput = 0.00;
}

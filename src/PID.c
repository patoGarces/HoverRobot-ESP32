#include "PID.h"
#include "stdio.h"

const uint8_t periodoPID=10;  										//periodo cada el cual se ejecuta el PID en ms

float angle_min, angle_max;
float ITerm, lastInput;
uint16_t contador_timer=0;

pid_control_t PID_n1;

void pidInit(pid_params_t params,float angleMin,float angleMax){																

	PID_n1.enablePID = 0;
	pidSetConstants(params.kp,params.ki,params.kd,params.centerAngle);
    pidSetLimits(angleMin,angleMax);
	// pidSetPointAngle(0);
}

void pidEnable(void){
	
	PID_n1.enablePID = 1;
}

void pidDisable(void){
	
	PID_n1.enablePID = 0;
}

/*
 * Esta funcion debe ser llamada cada un periodo fijo definido en periodoPID
 */
float pidCalculate(float input){
	
	if(!PID_n1.enablePID){
		 return 0;
	}
	PID_n1.input=input;
	double error = PID_n1.set_angle - PID_n1.input;										//calculo P: resto error entre el valor seteado y el de entrada 		
	ITerm+= (PID_n1.ki * error);														//calculo I: acumulo error ya multiplicado por ki

	if(ITerm> angle_max){																//recorto el termino integral maximo
		ITerm= angle_max;
	}
	else if(ITerm< angle_min){															//recorto el termino intgral minimo
		ITerm= angle_min;
	}
	double dInput = (PID_n1.input - lastInput);											//Calculo D: resto la entrada anterior a la actual

	/*Compute PID Output*/
	PID_n1.output = (PID_n1.kp * error) + ITerm - (PID_n1.kd * dInput);					//opero con los 3 parametros para obtener salida, el D se resta para evitar la kick derivate
	if(PID_n1.output > angle_max){														//recorto la salida máxima
		PID_n1.output = angle_max;
	}
	else if(PID_n1.output < angle_min){													//recorto la salida minima
		PID_n1.output = angle_min;
	}

	lastInput = PID_n1.input;

	return PID_n1.output;
}
 
void pidSetPointAngle(float angulo){
	PID_n1.set_angle=angulo;
}

/*
 * 	Funcion para cargar los parametros al filtro PID
 *	Todos los parametros deben estar entre 0.00 y 1.00
*/ 
void pidSetConstants(float KP,float KI,float KD,float targetAngle){
	pid_params_t writeParams = {0};

	double SampleTimeInSec = ((double)periodoPID)/1000;
	PID_n1.kp = KP;
	PID_n1.ki = KI * SampleTimeInSec;																 
	PID_n1.kd = KD / SampleTimeInSec;
	PID_n1.set_angle= targetAngle;

	writeParams.kp = KP;
	writeParams.ki = KI;
	writeParams.kd = KD;
	writeParams.centerAngle = targetAngle;

	printf("pidSetConstant : %f\n",KP);
  	storageWritePidParams(writeParams);
}
 
 /* 
 * Funcion para limitar la salida del Filtro PID
 * si se encuentra en runtime primero recorto los parametros PID
 * para adecuarlos a los nuevos maximos y minimos
 */
void pidSetLimits(float min, float max){	                // TODO: modificar para setear el minimo y el maximo de la salida, no un angulo???

    if(min > max){
        return;
    }
    angle_min = min;
    angle_max = max;

    if(PID_n1.output > angle_max){
        PID_n1.output = angle_max;
    }
    else if(PID_n1.output < angle_min){
        PID_n1.output = angle_min;
    }

    if(ITerm> angle_max){
        ITerm= angle_max;
    }
    else if(ITerm< angle_min){
        ITerm= angle_min;
    }
}


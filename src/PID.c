#include "PID.h"
#include "stdio.h"
#include "main.h"

float ITerm, lastInput;
uint16_t contador_timer=0;

double sampleTimeInSec = ((double)PERIOD_IMU_MS)/1000;

pid_control_t PID_n1;

void pidInit(pid_params_t params){															

	PID_n1.enablePID = 0;
	pidSetConstants(params.kp,params.ki,params.kd);
	pidSetPointAngle(params.center_angle);
}

void setEnablePid(void){ 
	PID_n1.enablePID = true;
}

void setDisablePid(void){
	PID_n1.enablePID = false;
}

bool getEnablePid(void){
	return PID_n1.enablePID;
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
	ITerm+= (sampleTimeInSec * error);													//calculo I: acumulo error ya multiplicado por ki

	if(ITerm > 15.00){																	//recorto el termino integral maximo
		ITerm = 15.00;
	}
	else if(ITerm < -15.00){															//recorto el termino intgral minimo
		ITerm = -15.00;
	}
	double dInput = (PID_n1.input - lastInput) / sampleTimeInSec;						//Calculo D: resto la entrada anterior a la actual

	/*Compute PID Output*/
	PID_n1.output = (PID_n1.kp * error) + ((PID_n1.ki/100)  * ITerm) + ((PID_n1.kd/100) * dInput);	//opero con los 3 parametros para obtener salida, el D se resta para evitar la kick derivate
	if(PID_n1.output > 1.00){															//recorto la salida máxima
		PID_n1.output = 1.00;
	}
	else if(PID_n1.output < -1.00){														//recorto la salida minima
		PID_n1.output = -1.00;
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
void pidSetConstants(float KP,float KI,float KD){

	PID_n1.kp = KP;
	PID_n1.ki = KI;																 
	PID_n1.kd = KD;
}

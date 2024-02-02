#include "PID.h"
#include "stdio.h"
#include "main.h"

float ITerm = 0.00, lastInput = 0.00;
float sampleTimeInSec = ((float)PERIOD_IMU_MS)/1000;

pid_control_t pidControl1;

float normalizeAngle( float angle );

void pidInit(pid_params_t params){															

	pidControl1.enablePID = 0;
	ITerm = 0.00;
	lastInput = 0.00;
	pidSetConstants(params.kp,params.ki,params.kd);
	pidSetPointAngle(params.center_angle);
}

void setEnablePid(void){ 
	pidControl1.enablePID = true;
}

void setDisablePid(void){
	pidControl1.enablePID = false;
}

bool getEnablePid(void){
	return pidControl1.enablePID;
}

float normalizeAngle( float angle ){
	return (angle / 100);
}

float cutNormalizeLimits( float normalizeInput ){
	if(normalizeInput > 1.00){																	//recorto el termino integral maximo
		return 1.00;
	}
	else if(normalizeInput < -1.00){																//recorto el termino intgral minimo
		return -1.00;
	}
	else{
		return normalizeInput;
	}
	
}

/*
 * Esta funcion debe ser llamada cada un periodo fijo definido en periodoPID
 * @param input en grados
 * @return resultado del PID normalizado 
 */
float pidCalculate( float input ){
	
	if( !pidControl1.enablePID ){
		 return 0;
	}
	pidControl1.input = normalizeAngle( input );
	float error = pidControl1.setPoint - pidControl1.input;							//calculo P: resto error entre el valor seteado y el de entrada 		
	ITerm += sampleTimeInSec * error * pidControl1.ki;									//calculo I: acumulo error ya multiplicado por ki
	float dInput = (pidControl1.input - lastInput) / sampleTimeInSec*100;				//Calculo D: resto la entrada anterior a la actual
	
	ITerm = cutNormalizeLimits( ITerm );
	dInput = cutNormalizeLimits( dInput );

	/*Compute PID Output*/
	pidControl1.output = (pidControl1.kp * error) + ITerm - (pidControl1.kd * dInput);	//opero con los 3 parametros para obtener salida, el D se resta para evitar la kick derivate
	
	pidControl1.output = cutNormalizeLimits( pidControl1.output );

	lastInput = pidControl1.input;
	return pidControl1.output;
}
 
void pidSetPointAngle(float angulo){
	pidControl1.setPoint= normalizeAngle( angulo );
}

/*
 * 	Funcion para cargar los parametros al filtro PID
 *	Todos los parametros deben estar entre 0.00 y 1.00
*/ 
void pidSetConstants(float KP,float KI,float KD){

	pidControl1.kp = KP;
	pidControl1.ki = KI;																 
	pidControl1.kd = KD;
	ITerm = 0.00;
	lastInput = 0.00;

	printf("SEEEEEEEEEEET CONSTAAAAAAAAAAAAAANTT KD: %f\n",pidControl1.kd);
}

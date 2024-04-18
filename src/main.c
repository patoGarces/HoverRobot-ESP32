#include "driver/gpio.h"
#include "soc/gpio_periph.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "stdio.h"
#include "math.h"

#include "driver/i2c.h"

#include "main.h"
#include "comms.h"
#include "PID.h"
#include "storage_flash.h"
#include "stepper.h"

#include "mpu6050_wrapper.h"
/* Incluyo componentes */
// #include "../components/MPU6050/include/MPU6050.h"
#include "../components/BT_CLASSIC/include/BT_CLASSIC.h"

#define GRAPH_ARDUINO_PLOTTER   false
#define MAX_VELOCITY            1000.00
#define VEL_MAX_CONTROL         5    
#define DEVICE_BT_NAME          "Balancing robot"

extern QueueSetHandle_t newAnglesQueue;                 // Recibo nuevos angulos obtenidos del MPU
QueueHandle_t queueNewPidParams;                        // Recibo nuevos parametros relacionados al pid
QueueSetHandle_t outputMotorQueue;                      // Envio nuevos valores de salida para el control de motores

QueueHandle_t queueReceiveControl;

status_robot_t statusToSend;                            // Estructura que contiene todos los parametros de status a enviar a la app

output_motors_t attitudeControlMotor;

int16_t testMotor=0;
// static void imuControlHandler(void *pvParameters){
//     float newAngles[3];
//     output_motors_t outputMotors;

//     outputMotorQueue = xQueueCreate(5,sizeof(output_motors_t));

//     pidSetEnable();

//     while(1){
//         if(xQueueReceive(newAnglesQueue,&newAngles,pdMS_TO_TICKS(10))){
//             statusToSend.roll = newAngles[AXIS_ANGLE_X];
//             statusToSend.pitch = newAngles[AXIS_ANGLE_Y];
//             statusToSend.yaw = newAngles[AXIS_ANGLE_Z];

//             // float roundedAngle = roundf(newAngles[AXIS_ANGLE_Y] * 10) / 10; 
//             float roundedAngle = newAngles[AXIS_ANGLE_Y]; 
//             uint16_t outputPidMotors = (uint16_t)(pidCalculate(roundedAngle) * MAX_VELOCITY); 

//             statusToSend.roundedAngle = roundedAngle;

//             // if(roundedAngle > 1.00 || roundedAngle < -1.00) {
//             //     outputMotors.motorL = outputPidMotors;
//             //     outputMotors.motorR = outputPidMotors;
//             // }
//             // else {
//             //     outputMotors.motorL = 0;
//             //     outputMotors.motorR = 0;
//             // }

//             // setVelMotors(outputMotors.motorL,outputMotors.motorR);

//             // statusToSend.speedL = outputMotors.motorL;
//             // statusToSend.speedR = outputMotors.motorR;

//             // if (!pidGetEnable()) { 
//             //     if (((newAngles[AXIS_ANGLE_Y] > (CENTER_ANGLE_MOUNTED-1)) && (newAngles[AXIS_ANGLE_Y] < (CENTER_ANGLE_MOUNTED+1)))) { 
//             //         pidSetEnable();   
//             //         statusToSend.status_code = STATUS_ROBOT_STABILIZED;                                                   
//             //     }
//             //     else {
//             //         pidSetDisable();
//             //     }
//             // }
//             // else { 
//             //     if (((newAngles[AXIS_ANGLE_Y] < (CENTER_ANGLE_MOUNTED-statusToSend.safetyLimits)) || (newAngles[AXIS_ANGLE_Y] > (CENTER_ANGLE_MOUNTED+statusToSend.safetyLimits)))){ 
//             //         setVelMotors(0,0);
//             //         pidSetDisable();
//             //         statusToSend.status_code = STATUS_ROBOT_DISABLE; 
//             //     }
//             // }
//             // gpio_set_level(PIN_OSCILO, 0);
//         }
//     }
// }

// static void updateParams(void *pvParameters){

//     pid_params_t newPidParams;

//     queueNewPidParams = xQueueCreate(1,sizeof(pid_params_t));

//     while (1){
        
//         if(xQueueReceive(queueNewPidParams,&newPidParams,0)){
//             newPidParams.center_angle += CENTER_ANGLE_MOUNTED;
//             pidSetConstants(newPidParams.kp,newPidParams.ki,newPidParams.kd);
//             pidSetSetPoint(newPidParams.center_angle);
//             storageWritePidParams(newPidParams);            

//             statusToSend.P = newPidParams.kp*100;  
//             statusToSend.I = newPidParams.ki*100;  
//             statusToSend.D = newPidParams.kd*100; 
//             statusToSend.centerAngle = newPidParams.center_angle;   
//             statusToSend.safetyLimits = newPidParams.safety_limits;   // TODO: incluir para enviarlo   

//             printf("Nuevos parametros!, safety limits: %f\n",newPidParams.safety_limits);              
//         }
//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }

// static void attitudeControl(void *pvParameters){

//     queueReceiveControl = xQueueCreate(1, sizeof(control_app_t));

//     control_app_t newControlVal;

//     while(true){
//         if( xQueueReceive(queueReceiveControl,
//                          &newControlVal,
//                          ( TickType_t ) 1 ) == pdPASS ){

//             attitudeControlMotor.motorL = newControlVal.axis_x * -1 * VEL_MAX_CONTROL + newControlVal.axis_y * -1 * VEL_MAX_CONTROL;
//             attitudeControlMotor.motorR = newControlVal.axis_x *  VEL_MAX_CONTROL + newControlVal.axis_y * -1 * VEL_MAX_CONTROL;
            
//             setVelMotors(attitudeControlMotor.motorL,attitudeControlMotor.motorR);

//             // if( !attitudeControlMotor.motorL && !attitudeControlMotor.motorR){
//             //     disableMotors();
//             // }
//             // else{
//             //     enableMotors();
//             // }

//             printf("CONTROL RECIBIDO: X: %ld, Y: %ld, motorL: %d ,motorR: %d \n",newControlVal.axis_x,newControlVal.axis_y,attitudeControlMotor.motorL,attitudeControlMotor.motorR);
//         } 
//     }
// }




#define PIN_SDA        32
#define PIN_CLK        33

quaternion_wrapper_t q;             // [w, x, y, z]         quaternion container
vector_float_wrapper_t gravity;     // [x, y, z]            gravity vector


// Quaternion q;           // [w, x, y, z]         quaternion container
// VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

void task_initI2C(void *ignore) {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	vTaskDelete(NULL);
}

void task_display(void*){
	mpu6050_initialize();
	mpu6050_dmpInitialize();    // retorna 0 si la inicializacion fue exitosa

	// This need to be setup individually
	// mpu.setXGyroOffset(220);
	// mpu.setYGyroOffset(76);
	// mpu.setZGyroOffset(-85);
	// mpu.setZAccelOffset(1788);
    mpu6050_calibrateAccel(6);
    mpu6050_calibrateGyro(6);

	mpu6050_setDMPEnabled(true);

	while(1){
	    mpuIntStatus = mpu6050_getIntStatus();
		// get current FIFO count
		fifoCount = mpu6050_getFIFOCount();

	    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
	        // reset so we can continue cleanly
	        mpu6050_resetFIFO();

	    // otherwise, check for DMP data ready interrupt frequently)
	    } else if (mpuIntStatus & 0x02) {
	        // wait for correct available data length, should be a VERY short wait
	        while (fifoCount < packetSize) fifoCount = mpu6050_getFIFOCount();

	        // read a packet from FIFO

	        mpu6050_getFIFOBytes(fifoBuffer, packetSize);
            mpu6050_dmpGetQuaternion(&q,fifoBuffer);
            mpu6050_dmpGetGravity(&gravity,&q);
            mpu6050_dmpGetYawPitchRoll(ypr,&q,&gravity);

            // printf("Quaterniones> w: %f, x:%f,y:%f,z:%f\n",q.w,q.x,q.y,q.z);
            // printf("gravity> x: %f, y:%f,z:%f\n",gravity.x,gravity.y,gravity.z);

	 		// mpu.dmpGetQuaternion(&q, fifoBuffer);
			// mpu.dmpGetGravity(&gravity, &q);
			// mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
			printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
			printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);
	    }

	    //Best result is to match with DMP refresh rate
	    // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
	    // Now its 0x13, which means DMP is refreshed with 10Hz rate
		// vTaskDelay(5/portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}








void app_main() {
    uint8_t cont1=0;
    pid_params_t readParams={0};

    gpio_set_direction(PIN_LED , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 0);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_OSCILO], PIN_FUNC_GPIO);
    gpio_set_direction(PIN_OSCILO , GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_OSCILO, 0);

    storageInit();

    btInit(DEVICE_BT_NAME);

    // mpu6050_init_t mpuConfig = {
    //     .sclGpio = GPIO_MPU_SCL,
    //     .sdaGpio = GPIO_MPU_SDA,
    //     .intGpio = GPIO_MPU_INT,
    //     .sampleTimeInMs = PERIOD_IMU_MS,
    //     .accelSensitivity = MPU_ACCEL_SENS_16G,
    //     .gyroSensitivity = MPU_GYRO_SENS_2000,
    //     .lowPassFilterValue = MPU_LOW_PASS_FILTER_5HZ,
    //     .priorityTask = MPU_HANDLER_PRIORITY
    // };
    // mpuInit(mpuConfig);
    // vTaskDelay(pdMS_TO_TICKS(5000));
    // mpuCalibrate();
    // readParams = storageReadPidParams();

    // readParams.kp = 1.00;
    // readParams.ki = 0.0;
    // readParams.kd = 0.3;
    // readParams.safety_limits = 50;
    // printf("center: %f kp: %f , ki: %f , kd: %f,safetyLimits: %f\n",readParams.center_angle,readParams.kp,readParams.ki,readParams.kd,readParams.safety_limits);
    
    // pid_init_t pidConfig = {
    //     .kp = readParams.kp,
    //     .ki = readParams.ki,
    //     .kd = readParams.kd,
    //     .initSetPoint = readParams.center_angle,
    //     .sampleTimeInMs = PERIOD_IMU_MS,
    // };
    // pidInit(pidConfig);

    statusToSend.safetyLimits = readParams.safety_limits;

    // setMicroSteps(true);
    // motorsInit();
    // setVelMotors(250,2  50);
    // enableMotors();

    // while(true);

    // for(uint16_t i=0;i<1000;i++){
    //     setVelMotors(i,i);
    //     printf("vel: %d\n",i);
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }

    // for(int i=1000;i>0;i--){
    //     setVelMotors(i,i);
    //     printf("vel: %d\n",i);
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }
    // setVelMotors(0,0);

    // while(true){
    //     // setMicroSteps(true);
    //     // vTaskDelay(1000);
    //     // setMicroSteps(false);
    //     vTaskDelay(1000);
    // }

    // xTaskCreatePinnedToCore(imuControlHandler,"Imu Control Task",4096,NULL,IMU_HANDLER_PRIORITY,NULL,IMU_HANDLER_CORE);
    // xTaskCreate(updateParams,"Update Params Task",2048,NULL,3,NULL);
    // xTaskCreate(attitudeControl,"attitude control Task",2048,NULL,4,NULL);


    xTaskCreatePinnedToCore(task_initI2C,"task_initi2c",4096,NULL,5,NULL,IMU_HANDLER_CORE);
    xTaskCreatePinnedToCore(task_display,"task_display",4096,NULL,5,NULL,IMU_HANDLER_CORE);
    while(true) {
        vTaskDelay(100);
    }

    setVelMotors(0,0);

    uint8_t flagInc=true;

    while(1){
        
        // if(btIsConnected()){
        //     cont1++;
        //     if(cont1>50){
        //         cont1 =0;
        //     }
        //     statusToSend.header = HEADER_COMMS;
        //     statusToSend.bat_voltage = 10;
        //     statusToSend.bat_percent = 55;
        //     statusToSend.batTemp = 100-cont1;
        //     statusToSend.temp_uc_control = cont1;
        //     statusToSend.temp_uc_main = 123-cont1; 
        //     // statusToSend.status_code = 0;
        //     sendStatus(statusToSend);
        // }
        // gpio_set_level(PIN_LED,1);
        vTaskDelay(pdMS_TO_TICKS(50));
        // gpio_set_level(PIN_LED,0);
        // vTaskDelay(pdMS_TO_TICKS(100));

        // printf(">angle:%f\n",statusToSend.speedL/100.0);


        // printf(">angle:%f\n",statusToSend.roundedAngle-80.0);
        // printf(">outputMotor:%f\n",statusToSend.speedL/100.0);

        printf(">angle:%f\n>outputMotor:%f\n",statusToSend.roundedAngle,statusToSend.speedL/100.0);

        // printf("angle:%f,velTest:%d,outputPidMotors:%d\n",roundedAngle,testMotor*10,outputMotors.motorL);
        // printf("angleX:%f,angleY:%f\n",newAngles[AXIS_ANGLE_X],newAngles[AXIS_ANGLE_Y]);

        // if(GRAPH_ARDUINO_PLOTTER){
            //Envio log para graficar en arduino serial plotter
            // printf("angle_x:%f,set_point: %f,output_pid: %f, output_motor:%d\n",newAngles[AXIS_ANGLE_Y],CENTER_ANGLE_MOUNTED,outputPid,outputMotors.motorL);
        // }

        if(flagInc){
            testMotor++;
            if(testMotor>99){
                flagInc=false;
            }
        }
        else {
            testMotor--;
            if(testMotor < -99){
                flagInc=true;
                setVelMotors(0,0);
            }
        }
        statusToSend.speedL = testMotor*10;
        statusToSend.speedR = testMotor*10;
        setVelMotors(statusToSend.speedL,statusToSend.speedR);
    }
}
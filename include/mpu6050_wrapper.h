#ifndef MPU6050_WRAPPER_H
#define MPU6050_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    gpio_num_t sclGpio;
    gpio_num_t sdaGpio;
    gpio_num_t intGpio;
    uint8_t priorityTask;
    uint8_t core;
} mpu6050_init_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_wrapper_t;

typedef struct {
    float x;
    float y;
    float z;
} vector_float_wrapper_t;

typedef struct {
    float yaw;
    float pitch;
    float roll;
    float temp;
} vector_queue_t;

enum {
    AXIS_ANGLE_X,
    AXIS_ANGLE_Y,
    AXIS_ANGLE_Z
};

void mpu6050_initialize(mpu6050_init_t *config);
// int mpu6050_testConnection();
void mpu6050_recalibrate();
// int mpu6050_dmpInitialize();
// void mpu6050_calibrateAccel(int loops);
// void mpu6050_calibrateGyro(int loops);
// void mpu6050_setDMPEnabled(bool enable);
// void mpu6050_DMPReset();

// uint8_t mpu6050_getIntStatus();
// uint16_t mpu6050_getFIFOCount();
// void mpu6050_resetFIFO();
// void mpu6050_getFIFOBytes(uint8_t *data, uint8_t length);
// void mpu6050_dmpGetQuaternion(quaternion_wrapper_t *q, const uint8_t* packet);
// uint8_t mpu6050_dmpGetGravity(vector_float_wrapper_t *v, const quaternion_wrapper_t *q);
// void mpu6050_dmpGetYawPitchRoll(float *ypr, const quaternion_wrapper_t *q, const vector_float_wrapper_t *gravity);
// void mpu6050_getAcceleration(int16_t* ax, int16_t* ay, int16_t* az);
// void mpu6050_getRotation(int16_t* gx, int16_t* gy, int16_t* gz);
// int16_t mpu6050_getTemperature();
// void mpu6050_setAccelRange(uint8_t range);
// void mpu6050_setGyroRange(uint8_t range);
// void mpu6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_WRAPPER_H

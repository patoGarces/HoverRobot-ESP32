#ifndef MPU6050_WRAPPER_H
#define MPU6050_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void mpu6050_init();
int mpu6050_testConnection();
void mpu6050_getAcceleration(int16_t* ax, int16_t* ay, int16_t* az);
void mpu6050_getRotation(int16_t* gx, int16_t* gy, int16_t* gz);
int16_t mpu6050_getTemperature();
void mpu6050_setAccelRange(uint8_t range);
void mpu6050_setGyroRange(uint8_t range);
void mpu6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_WRAPPER_H

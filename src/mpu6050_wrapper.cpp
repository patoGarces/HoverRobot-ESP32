// #include "MPU6050.h"
#include "../components/MPU6050/MPU6050.h"
#include "mpu6050_wrapper.h"

#ifdef __cplusplus
extern "C" {
#endif

static MPU6050 mpu;

void mpu6050_init() {
    mpu.initialize();
}

int mpu6050_testConnection() {
    return mpu.testConnection() ? 1 : 0;
}

void mpu6050_getAcceleration(int16_t* ax, int16_t* ay, int16_t* az) {
    mpu.getAcceleration(ax, ay, az);
}

void mpu6050_getRotation(int16_t* gx, int16_t* gy, int16_t* gz) {
    mpu.getRotation(gx, gy, gz);
}

int16_t mpu6050_getTemperature() {
    return mpu.getTemperature();
}

void mpu6050_setAccelRange(uint8_t range) {
    mpu.setFullScaleAccelRange(range);
}

void mpu6050_setGyroRange(uint8_t range) {
    mpu.setFullScaleGyroRange(range);
}

void mpu6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    mpu.getMotion6(ax, ay, az, gx, gy, gz);
}

#ifdef __cplusplus
}
#endif

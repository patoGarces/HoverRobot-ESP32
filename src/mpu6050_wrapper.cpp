#include "../components/MPU6050/MPU6050.h"
#include "../components/MPU6050/MPU6050_6Axis_MotionApps20.h"

#include "mpu6050_wrapper.h"

#ifdef __cplusplus
extern "C" {
#endif

static MPU6050 mpu;

void mpu6050_initialize() {
    mpu.initialize();
}

int mpu6050_testConnection() {
    return mpu.testConnection() ? 1 : 0;
}

int mpu6050_dmpInitialize() {
    return mpu.dmpInitialize();
}

void mpu6050_calibrateAccel(int loops) {
    mpu.CalibrateAccel(loops);
}

void mpu6050_calibrateGyro(int loops) {
    mpu.CalibrateGyro(loops);
}

void mpu6050_setDMPEnabled(bool enable) {
    mpu.setDMPEnabled(enable);
}

uint8_t mpu6050_getIntStatus() {
    return mpu.getIntStatus();
}

uint16_t mpu6050_getFIFOCount() {
    return mpu.getFIFOCount();
}

void mpu6050_resetFIFO() {
    mpu.resetFIFO();
}

void mpu6050_getFIFOBytes(uint8_t *data, uint8_t length) {  // TODO: ojo los punteros!
    mpu.getFIFOBytes(data,length);
}

void mpu6050_dmpGetQuaternion(quaternion_wrapper_t *q, const uint8_t* packet) {
    mpu.dmpGetQuaternion(reinterpret_cast<Quaternion*>(q), packet);
}

uint8_t mpu6050_dmpGetGravity(vector_float_wrapper_t *v, const quaternion_wrapper_t *q) {
    // VectorFloat gravityVector;
    
    // Copia el cuaternión a uno no constante
    quaternion_wrapper_t q_non_const = *q;
    
    // Llama al método correspondiente de la clase MPU6050
    uint8_t status = mpu.dmpGetGravity(reinterpret_cast<VectorFloat*>(v), reinterpret_cast<Quaternion*>(&q_non_const));
    
    // Copia los valores de la gravedad al wrapper de vector_float_wrapper_t
    // v->x = gravityVector.x;
    // v->y = gravityVector.y;
    // v->z = gravityVector.z;
    
    return status;
}

// uint8_t mpu6050_dmpGetGravity(vector_float_wrapper_t *v, const quaternion_wrapper_t *q) {
//     MPU6050 mpu;
//     VectorFloat gravityVector;
    
//     // Copia el cuaternión a uno no constante
//     quaternion_wrapper_t q_non_const = *q;
    
//     // Llama al método correspondiente de la clase MPU6050
//     uint8_t status = mpu.dmpGetGravity(&gravityVector, reinterpret_cast<Quaternion*>(&q_non_const));
    
//     // Copia los valores de la gravedad al wrapper de vector_float_wrapper_t
//     v->x = gravityVector.x;
//     v->y = gravityVector.y;
//     v->z = gravityVector.z;
    
//     return status;
// }

// void mpu6050_dmpGetYawPitchRoll(float *ypr, const quaternion_wrapper_t *q, const vector_float_wrapper_t *gravity) {
//     quaternion_wrapper_t q_non_const = *q;
//     // Llama a los métodos correspondientes de la clase MPU6050
//     mpu.dmpGetYawPitchRoll(ypr, reinterpret_cast<Quaternion*>(&q_non_const), reinterpret_cast<const VectorFloat*>(gravity));
// }
// void mpu6050_dmpGetYawPitchRoll(float *ypr, const quaternion_wrapper_t *q, const vector_float_wrapper_t *gravity) {
//     quaternion_wrapper_t q_non_const = *q;
//     // Accede al objeto VectorFloat desreferenciando directamente el puntero gravity
//     const VectorFloat gravityVector = *reinterpret_cast<const VectorFloat*>(gravity);
//     // Llama a los métodos correspondientes de la clase MPU6050
//     mpu.dmpGetYawPitchRoll(ypr, reinterpret_cast<Quaternion*>(&q_non_const), &gravityVector);
// }
void mpu6050_dmpGetYawPitchRoll(float *ypr, const quaternion_wrapper_t *q, const vector_float_wrapper_t *gravity) {
    quaternion_wrapper_t q_non_const = *q;
    // Accede al objeto VectorFloat desreferenciando directamente el puntero gravity
    const VectorFloat gravityVector = *reinterpret_cast<const VectorFloat*>(gravity);
    // Llama a los métodos correspondientes de la clase MPU6050
    mpu.dmpGetYawPitchRoll(ypr, reinterpret_cast<Quaternion*>(&q_non_const), const_cast<VectorFloat*>(&gravityVector));
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

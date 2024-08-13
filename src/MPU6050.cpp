/*!
 * @file MPU6050.cpp
 *
 * @mainpage MPU6050 Sensors
 * 
 * @section Intoduction
 * 
 * This is a library for the MPU6050 accelerometer, gyroscope 
 * and temperature sensor
 *
 * These displays use I2C to communicate, 2 pins are required to
 * interface
 * 
 * @section Author
 * 
 * Written by Corentin Roche
 * 
 */

#include "INSARIANNE_MPU6050.h"


MPU6050::MPU6050() : I2C() {
    _addr = MPU_ADDR;
}

bool MPU6050::begin(uint8_t para_gyr, uint8_t para_acc) {
    if(!write(MPU6050_FSYNC_OUT_DISABLED, 1, true, MPU6050_SMPLRT_DIV, 1) ||
       !write_bits(para_gyr, MPU6050_GYRO_CONFIG, 2, 3) ||
       !write_bits(para_acc, MPU6050_ACCEL_CONFIG, 2, 3))
            return false;

    switch(read_bits(0x1B, 2, 3)) {
        case 0b00:
          gyro_scale = 131; break;
        case 0b01:
          gyro_scale = 65.5; break;
        case 0b10:
          gyro_scale = 32.8; break;
        case 0b11:
          gyro_scale = 16.4; break;
        default:
          return false;
    }

      switch(read_bits(0x1C, 2, 3)) {
        case 0b00:
          accel_scale = 16384; break;
        case 0b01:
          accel_scale = 8192; break;
        case 0b10:
          accel_scale = 4096; break;
        case 0b11:
          accel_scale = 2048; break;
        default:
          return false;
    }
    return true;
}

bool MPU6050::begin(void) {
    return begin(MPU6050_RANGE_2_G, MPU6050_RANGE_500_DEG);
}

void MPU6050::read_sensor(void) {
    uint8_t buffer[14];

    if (!read_n(MPU6050_ACCEL_OUT, buffer, 14)) 
        return false;

    rawAccX = buffer[0] << 8 | buffer[1];
    rawAccY = buffer[2] << 8 | buffer[3];
    rawAccZ = buffer[4] << 8 | buffer[5];
    
    rawTemp = buffer[6] << 8 | buffer[7];

    rawGyroX = buffer[8] << 8 | buffer[9];
    rawGyroY = buffer[10] << 8 | buffer[11];
    rawGyroY = buffer[12] << 8 | buffer[13];


    temperature = (rawTemp / 340.0) + 36.53;

    accX = ((float)rawAccX) / accel_scale;
    accY = ((float)rawAccY) / accel_scale;
    accZ = ((float)rawAccZ) / accel_scale;

    gyroX = ((float)rawGyroX) / gyro_scale;
    gyroY = ((float)rawGyroY) / gyro_scale;
    gyroZ = ((float)rawGyroZ) / gyro_scale;

    return true;
}

void MPU6050::read_acce(void) {
    uint8_t buffer[6];

    if (!read_n(MPU6050_ACCEL_OUT, buffer, 6)) 
        return false;

    rawAccX = buffer[0] << 8 | buffer[1];
    rawAccY = buffer[2] << 8 | buffer[3];
    rawAccZ = buffer[4] << 8 | buffer[5];

    return true;
}

void MPU6050::read_gyro(void) {
    uint8_t buffer[6];

    if (!read_n(MPU6050_GYRO_OUT, buffer, 6)) 
        return false;

    rawGyroX = buffer[0] << 8 | buffer[1];
    rawGyroY = buffer[2] << 8 | buffer[3];
    rawGyroY = buffer[4] << 8 | buffer[5];

    gyroX = ((float)rawGyroX) / gyro_scale;
    gyroY = ((float)rawGyroY) / gyro_scale;
    gyroZ = ((float)rawGyroZ) / gyro_scale;

    return true;
}

void MPU6050::read_temp(void) {
    uint8_t buffer[2];

    if (!read_n(MPU6050_TEMP_OUT, buffer, 2)) 
        return false;
    
    rawTemp = buffer[0] << 8 | buffer[1];

    temperature = (rawTemp / 340.0) + 36.53;

    return true;
}

void MPU6050::Set_gyro_scale(float new_scale){
    gyro_scale = new_scale;
}

void MPU6050::Set_accel_scale(float new_scale){
    accel_scale = new_scale;
}

bool MPU6050::Set_param_register(uint8_t val, uint8_t reg) {
    return write(val, 1, true, reg, 1);
}

bool MPU6050::Set_param_register_bits(uint8_t val, uint8_t reg, uint8_t bits, uint8_t shift){
    return write_bits(val, reg, bits, shift);
}

float MPU6050::Get_gyro_scale(void) {
    return gyro_scale;
}

float MPU6050::Get_accel_scale(void) {
    return accel_scale;
}

uint8_t MPU6050::Get_param_register(uint8_t reg) {
    return read8(reg);
}

uint8_t MPU6050::Get_param_register(uint8_t reg, uint8_t bits, uint8_t shift) {
    return read_bits(reg, bits, shift);
}

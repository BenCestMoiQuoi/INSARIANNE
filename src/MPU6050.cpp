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

#include "INSARIANNE.h"


#define MPU_ADDR 0x68

#define MPU6050_SELF_TEST_X 0x0D ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Y 0x0E ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Z 0x0F ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_A 0x10 ///< Self test factory calibrated values register
#define MPU6050_SMPLRT_DIV 0x19  ///< sample rate divisor register
#define MPU6050_CONFIG 0x1A      ///< General configuration register
#define MPU6050_GYRO_CONFIG 0x1B ///< Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG 0x1C ///< Accelerometer specific configration register
#define MPU6050_GYRO_OUT 0x43   ///< Gyroscope data register
#define MPU6050_TEMP_OUT 0x41     ///< Temperature data register
#define MPU6050_ACCEL_OUT 0x3B  ///< Accelerometer data register

#define MPU6050_FSYNC_OUT_DISABLED 0b000
#define MPU6050_FSYNC_OUT_TEMP 0b001
#define MPU6050_FSYNC_OUT_GYROX 0b010
#define MPU6050_FSYNC_OUT_GYROY 0b011
#define MPU6050_FSYNC_OUT_GYROZ 0b100
#define MPU6050_FSYNC_OUT_ACCELX 0b101
#define MPU6050_FSYNC_OUT_ACCELY 0b110
#define MPU6050_FSYNC_OUT_ACCEL_Z 0b111

#define MPU6050_RANGE_2_G 0b00  ///< +/- 2g (default value)
#define MPU6050_RANGE_4_G = 0b01  ///< +/- 4g
#define MPU6050_RANGE_8_G = 0b10  ///< +/- 8g
#define MPU6050_RANGE_16_G = 0b11 ///< +/- 16g

#define MPU6050_RANGE_250_DEG 0b00  ///< +/- 250 deg/s (default value)
#define MPU6050_RANGE_500_DEG 0b01  ///< +/- 500 deg/s
#define MPU6050_RANGE_1000_DEG 0b10 ///< +/- 1000 deg/s
#define MPU6050_RANGE_2000_DEG 0b11 ///< +/- 2000 deg/s

#define MPU6050_BAND_260_HZ 0b000 ///< Docs imply this disables the filter
#define MPU6050_BAND_184_HZ 0b001 ///< 184 Hz
#define MPU6050_BAND_94_HZ 0b010  ///< 94 Hz
#define MPU6050_BAND_44_HZ 0b011  ///< 44 Hz
#define MPU6050_BAND_21_HZ 0b100  ///< 21 Hz
#define MPU6050_BAND_10_HZ 0b101  ///< 10 Hz
#define MPU6050_BAND_5_HZ 0b110   ///< 5 H


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

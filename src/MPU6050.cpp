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
#define MPU6050_INT_PIN_CONFIG 0x37 ///< Interrupt pin configuration register
#define MPU6050_INT_ENABLE 0x38     ///< Interrupt enable configuration register
#define MPU6050_INT_STATUS 0x3A     ///< Interrupt status register
#define MPU6050_WHO_AM_I 0x75       ///< Divice ID register
#define MPU6050_SIGNAL_PATH_RESET 0x68 ///< Signal path reset register
#define MPU6050_USER_CTRL 0x6A         ///< FIFO and I2C Master control register
#define MPU6050_PWR_MGMT_1 0x6B ///< Primary power/sleep control register
#define MPU6050_PWR_MGMT_2 0x6C ///< Secondary power/sleep control register
#define MPU6050_TEMP_H 0x41     ///< Temperature data high byte register
#define MPU6050_TEMP_L 0x42     ///< Temperature data low byte register
#define MPU6050_ACCEL_OUT 0x3B  ///< base address for sensor data reads
#define MPU6050_MOT_THR 0x1F    ///< Motion detection threshold bits [7:0]
#define MPU6050_MOT_DUR 0x20 ///< Duration counter threshold for motion int. 1 kHz rate, LSB = 1 ms

#define MPU6050_FSYNC_OUT_DISABLED 0
#define MPU6050_FSYNC_OUT_TEMP 1
#define MPU6050_FSYNC_OUT_GYROX 2
#define MPU6050_FSYNC_OUT_GYROY 3
#define MPU6050_FSYNC_OUT_GYROZ 4
#define MPU6050_FSYNC_OUT_ACCELX 5
#define MPU6050_FSYNC_OUT_ACCELY 6
#define MPU6050_FSYNC_OUT_ACCEL_Z 7

#define MPU6050_INTR_8MHz 0
#define MPU6050_PLL_GYROX 1
#define MPU6050_PLL_GYROY 2
#define MPU6050_PLL_GYROZ 3
#define MPU6050_PLL_EXT_32K 4
#define MPU6050_PLL_EXT_19MHz 5
#define MPU6050_STOP 7

#define MPU6050_RANGE_2_G 0b00  ///< +/- 2g (default value)
#define MPU6050_RANGE_4_G = 0b01  ///< +/- 4g
#define MPU6050_RANGE_8_G = 0b10  ///< +/- 8g
#define MPU6050_RANGE_16_G = 0b11 ///< +/- 16g

#define MPU6050_RANGE_250_DEG 0  ///< +/- 250 deg/s (default value)
#define MPU6050_RANGE_500_DEG 1  ///< +/- 500 deg/s
#define MPU6050_RANGE_1000_DEG 2 ///< +/- 1000 deg/s
#define MPU6050_RANGE_2000_DEG 3 ///< +/- 2000 deg/s

#define MPU6050_BAND_260_HZ 0 ///< Docs imply this disables the filter
#define MPU6050_BAND_184_HZ 1 ///< 184 Hz
#define MPU6050_BAND_94_HZ 2  ///< 94 Hz
#define MPU6050_BAND_44_HZ 3  ///< 44 Hz
#define MPU6050_BAND_21_HZ 4  ///< 21 Hz
#define MPU6050_BAND_10_HZ 5  ///< 10 Hz
#define MPU6050_BAND_5_HZ 6   ///< 5 Hz

#define MPU6050_HIGHPASS_DISABLE 0
#define MPU6050_HIGHPASS_5_HZ 1
#define MPU6050_HIGHPASS_2_5_HZ 2
#define MPU6050_HIGHPASS_1_25_HZ 3
#define MPU6050_HIGHPASS_0_63_HZ 4
#define MPU6050_HIGHPASS_UNUSED 5
#define MPU6050_HIGHPASS_HOLD 6

#define MPU6050_CYCLE_1_25_HZ 0 ///< 1.25 Hz
#define MPU6050_CYCLE_5_HZ 1    ///< 5 Hz
#define MPU6050_CYCLE_20_HZ 2  ///< 20 Hz
#define MPU6050_CYCLE_40_HZ 3  ///< 40 Hz


MPU6050::MPU6050() : I2C() {
    _addr = MPU_ADDR;
}

bool MPU6050::begin() {
    write(MPU6050_FSYNC_OUT_DISABLED, 1, true, MPU6050_SMPLRT_DIV, 1);
    write_bits(MPU6050_BAND_260_HZ, MPU6050_ACCEL_CONFIG, 2, 3);
    write_bits(MPU6050_RANGE_500_DEG, MPU6050_GYRO_CONFIG, 2, 3);
    write_bits(MPU6050_RANGE_2_G, MPU6050_ACCEL_CONFIG, 2, 3);

    gyro_scale = 65.5;
    accel_scale = 16384;
    return true;
}

void MPU6050::read_sensor(void) {
    uint8_t buffer[14];

    read_n((uint8_t)MPU6050_ACCEL_OUT, buffer, 14);

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
}
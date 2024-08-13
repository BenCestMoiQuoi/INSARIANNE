/*


*/



#ifndef INSARIANNE_MPU6050_H
#define INSARIANNE_MPU6050_H

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


class MPU6050 : private I2C
{
  public:
    MPU6050();

    bool begin();
    bool begin(uint8_t para_gyr, uint8_t para_acc);
    void read_sensor(void);

    void read_acce(void);
    void read_gyro(void);
    void read_temp(void);
    
    void Set_gyro_scale(float new_scale);
    void Set_accel_scale(float new_scale);
    bool Set_param_register(uint8_t val, uint8_t reg);
    bool Set_param_register_bits(uint8_t val, uint8_t reg, uint8_t bits, uint8_t shift);
    float Get_gyro_scale(void);
    float Get_accel_scale(void);
    uint8_t Get_param_register(uint8_t reg);
    uint8_t Get_param_register(uint8_t reg, uint8_t bits, uint8_t shift);

    float temperature, gyroX, gyroY, gyroZ, accX, accY, accZ;

  private:
    int _sensorID_tem = 0x650, _sensorID_acc = 0x651, _sensorID_gyr = 0x652;
    int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
    float accel_scale, gyro_scale;
};

#endif
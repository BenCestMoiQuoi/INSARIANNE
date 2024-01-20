/*!
 * @file INSARIANNE.h
 *
 * This is a library for the BMP085 Barometric Pressure + Temp
 * sensor and the MUP6050 accelerometer, gyroscopique and temperature sensor
 *
 * These displays use I2C to communicate, 2 pins are required to
 * interface
 * 
 */

#ifndef INSARIANNE_H
#define INSARIANNE_H

#include <Arduino.h>
#include <Wire.h>
#include "Register.h"


class sensor {
  protected:
    sensor(TwoWire *theWire = &Wire);

    bool write(uint8_t *data, size_t len, bool stop = true, \
               uint8_t *pre_data = nullptr, size_t pre_len = 0);
    uint8_t read8(uint8_t *reg);
    uint16_t read16(uint8_t *reg);
    bool read_n(uint8_t *reg, uint8_t data[], int n);
    bool write_bits(uint8_t *data, uint8_t *reg, uint8_t bits, uint8_t shift);
    uint8_t read_bits(uint8_t *reg, uint8_t bits, uint8_t shift);
  
    uint8_t _addr;
    TwoWire *_wire;
};


class BMP085 : private sensor
{
  public:
    BMP085();

    bool begin();

    void read_sensor(void);
    void Set_SLP(float altitude_base);

    float pressure, sealevelpressure, temperature, altitude;

  private:
    void Temperature(void);
    void Pressure(void);
    void Altitude(void);

    uint8_t oversampling;
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
    int32_t UP, UT;
    int32_t c3, c5, c6, x1, x2, x3;
    uint32_t c4, c7, c1;
};


class MPU6050 : private sensor
{
  public:
    MPU6050();

    bool begin();
    void read_sensor(void);

    float temperature, gyroX, gyroY, gyroZ, accX, accY, accZ;

  private:
    int _sensorID_tem = 0x650, _sensorID_acc = 0x651, _sensorID_gyr = 0x652;
    int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
    float accel_scale, gyro_scale;
};


#endif 

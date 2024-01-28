#ifndef INSARIANNE_2_H
#define INSARIANNE_2_H

#include <Arduino.h>
#include <Wire.h>

#define VERSION_LIB_2 "1.1.5"


class I2C {
  protected:
    I2C(TwoWire *theWire = &Wire) {
      _wire = theWire;
    }
    void begin(uint8_t address){
      _addr = address;
    }

    bool write(uint8_t *data, size_t len, bool stop = true, \
               uint8_t *reg = nullptr, size_t reg_len = 0) {
      _wire->beginTransmission(_addr);
      if ((reg_len != 0) && (reg != nullptr)) {
        if (_wire->write(reg, reg_len) != reg_len) return false;
      }
      if((len != 0) && (data != nullptr)) {
        if (_wire->write(data, len) != len) return false;
      }
      if (_wire->endTransmission(stop) == 0) return true;
      return false;    
    }

    uint8_t read8(uint8_t *reg) {
      uint8_t buffer;
      if (read_n(reg, buffer, 2)) 
        return false;
      
      return buffer;
    }

    uint16_t read16(uint8_t *reg) {
      uint8_t buffer[2];
      
      if (read_n(reg, buffer, 2)) 
        return false;
      
      buffer[0] = _wire->read();
      buffer[1] = _wire->read();

      return (buffer[0] << 8 | buffer[1]);
    }

    bool read_n(uint8_t *reg, uint8_t data[], int n) {
      if (_wire->requestFrom(_addr, (uint8_t) n, reg, 1, true) != n) 
        return false;

      for (int i=0; i<n; i++) 
        data[i] = _wire->read();
      
      return true;
    }

    bool write_bits(uint8_t *data, uint8_t *reg, uint8_t bits, uint8_t shift) {
      uint8_t val = read8(reg);

      uint8_t mask = (1 << bits) - 1;
      *data &= mask;
      mask <<= shift;
      val &= ~mask;
      val |= *data << shift;

      write(val, 1, true, reg, 1);
    }

    uint8_t read_bits(uint8_t *reg, uint8_t bits, uint8_t shift) {
      uint8_t val = read8(reg);

      val >>= shift;
      return val & ((1 << bits) - 1);
    }

  
    uint8_t _addr;
    TwoWire *_wire;
};

#define MPU6050_ADRESS 0x68
#define MPU6050_REGISTRE 0x3B

class MPU6050 : private I2C
{
  public:
    MPU6050() : I2C() {
      _addr = MPU6050_ADRESS;
    }

    bool begin() {
      // write(MPU6050_FSYNC_OUT_DISABLED, 1, true, MPU6050_SMPLRT_DIV, 1);
      // write_bits(MPU6050_BAND_260_HZ, MPU6050_ACCEL_CONFIG, 2, 3);
      // write_bits(MPU6050_RANGE_500_DEG, MPU6050_GYRO_CONFIG, 2, 3);
      // write_bits(MPU6050_RANGE_2_G, MPU6050_ACCEL_CONFIG, 2, 3);

      gyro_scale = 131;
      accel_scale = 16384;
      return true;
    }

    void read_sensor(void) {
      uint8_t buffer[14];

      MPU6050::read_n(MPU6050_REGISTRE, buffer, 14);

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

    float temperature, gyroX, gyroY, gyroZ, accX, accY, accZ;

  private:
    int _sensorID_tem = 0x650, _sensorID_acc = 0x651, _sensorID_gyr = 0x652;
    int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;
    float accel_scale, gyro_scale;
};

#endif
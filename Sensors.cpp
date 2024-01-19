/*!
 * @file Sensors.h
 *
 * @mainpage INSARIANNE Sensors
 * 
 * @section Intoduction
 * 
 * This is a library for the BMP085 Barometric Pressure + Temp
 * sensor and the MUP6050
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
#include "Register.h"

//sensor::sensor() {}

bool sensor::write(uint8_t *data, size_t len, bool stop, 
                   uint8_t *reg, size_t reg_len) {
    _wire->beginTransmission(_addr);
    if ((reg_len != 0) && (reg != nullptr)) {
        if (_wire->write(reg, reg_len) != reg_len) {
            return false;
        }
    }
    if (_wire->write(data, len) != len) {
        return false;
    }
    if (_wire->endTransmission(stop) == 0) {
        return true;
    }
    return false;    
}

uint8_t sensor::read8(uint8_t *reg) {
    if (!write(reg, 1, true)) 
        return false;
    
    return _wire->read();
}

uint16_t sensor::read16(uint8_t *reg) {
    uint8_t buffer[2];
    
    if (!write(reg, 1, true)) 
        return false;
    
    buffer[0] = _wire->read();
    buffer[1] = _wire->read();

    return (buffer[0] << 8 | buffer[1]);
}

bool sensor::read_n(uint8_t *reg, uint8_t data[], int n) {
    if (!write(reg, 1, true)) 
        return false;

    for (int i=0; i<n; i++) 
        data[i] = _wire->read();
    
    return true;
}

bool sensor::write_bits(uint8_t *data, uint8_t *reg, uint8_t bits, uint8_t shift) {
    uint8_t val = read8(reg);

    uint8_t mask = (1 << bits) - 1;
    *data &= mask;
    mask <<= shift;
    val &= ~mask;
    val |= *data << shift;

    write(val, 1, true, reg, 1);
}

uint8_t sensor::read_bits(uint8_t *reg, uint8_t bits, uint8_t shift) {
    uint8_t val = read8(reg);

    val >>= shift;
    return val & ((1 << bits) - 1);
}



BMP085::BMP085() {
    _addr = BMP_ADDR;
}

bool BMP085::begin(TwoWire *theWire) {
    _wire = theWire;
    oversampling = BMP085_ULTRAHIGHRES;
    
    if (read8(0xD0) != 0x55)
        return false;

    /* read calibration data */
    ac1 = read16(BMP085_CAL_AC1);
    ac2 = read16(BMP085_CAL_AC2);
    ac3 = read16(BMP085_CAL_AC3);
    ac4 = read16(BMP085_CAL_AC4);
    ac5 = read16(BMP085_CAL_AC5);
    ac6 = read16(BMP085_CAL_AC6);

    b1 = read16(BMP085_CAL_B1);
    b2 = read16(BMP085_CAL_B2);

    mb = read16(BMP085_CAL_MB);
    mc = read16(BMP085_CAL_MC);
    md = read16(BMP085_CAL_MD);

    return true;
}

void BMP085::read_sensor(void) {
    Temperature();
    Pressure();
    Altitude();
}

void BMP085::Set_SLP(float altitude_base) {
    Temperature();
    Pressure();

    sealevelpressure = pressure / pow(1.0 - altitude_base / 44330, 5.255);
}

void BMP085::Temperature(void) {
    write(BMP085_READTEMPCMD, 1, true, BMP085_CONTROL, 1);
    UT = read16(BMP085_TEMPDATA);
    
    x1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
    x2 = ((int32_t)mc << 11) / (x1 + (int32_t)md);
    
    temperature = (x1 + x2 + 8) >> 4;
    temperature /= 10;
}

void BMP085::Pressure(void) {
    write(BMP085_READPRESSURECMD + (oversampling << 6), 1, true, BMP085_CONTROL, 1);
    
    if (oversampling == BMP085_ULTRALOWPOWER)
        delay(5);
    else if (oversampling == BMP085_STANDARD)
        delay(8);
    else if (oversampling == BMP085_HIGHRES)
        delay(14);
    else
        delay(26);

    c1 = read16(BMP085_PRESSUREDATA);

    c1 <<= 8;
    c1 |= read8(BMP085_PRESSUREDATA + 2);
    c1 >>= (8 - oversampling); 


    c6 = c5 - 4000;
    x1 = ((int32_t)b2 * ((c6 * c6) >> 12)) >> 11;
    x2 = ((int32_t)ac2 * c6) >> 11;
    x3 = x1 + x2;
    c3 = ((((int32_t)ac1 * 4 + x3) << oversampling) + 2) / 4;

    x1 = ((int32_t)ac3 * c6) >> 13;
    x2 = ((int32_t)b1 * ((c6 * c6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    c4 = ((uint32_t)ac4 * (uint32_t)(x3 + 32768)) >> 15;
    c7 = ((uint32_t)UP - c3) * (uint32_t)(50000UL >> oversampling);

    if (c7 < 0x80000000) {
        x3 = (c7 * 2) / c4;
    } else {
        x3 = (c7 / c4) * 2;
    }
    x1 = (x3 >> 8) * (x3 >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * x3) >> 16;

    pressure = x3 + ((x1 + x2 + (int32_t)3791) >> 4);
}

void BMP085::Altitude(void) {
    altitude = 44330 * (1.0 - pow(pressure / sealevelpressure, 0.1903));
}

MPU6050::MPU6050() {
    _addr = MPU_ADDR;
}

bool MPU6050::begin(TwoWire *theWire) {
    _wire = theWire;

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

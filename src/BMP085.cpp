/*!
 * @file BMP085.cpp
 *
 * @mainpage BMP085 Sensors
 * 
 * @section Intoduction
 * 
 * This is a library for the BMP085 Barometric Pressure + Temp sensor
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


#define BMP_ADDR 0x77

#define BMP085_ULTRALOWPOWER 0 //!< Ultra low power mode
#define BMP085_STANDARD 1      //!< Standard mode
#define BMP085_HIGHRES 2       //!< High-res mode
#define BMP085_ULTRAHIGHRES 3  //!< Ultra high-res mode
#define BMP085_CAL_AC1 0xAA    //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC2 0xAC    //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC3 0xAE    //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC4 0xB0    //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC5 0xB2    //!< R   Calibration data (16 bits)
#define BMP085_CAL_AC6 0xB4    //!< R   Calibration data (16 bits)
#define BMP085_CAL_B1 0xB6     //!< R   Calibration data (16 bits)
#define BMP085_CAL_B2 0xB8     //!< R   Calibration data (16 bits)
#define BMP085_CAL_MB 0xBA     //!< R   Calibration data (16 bits)
#define BMP085_CAL_MC 0xBC     //!< R   Calibration data (16 bits)
#define BMP085_CAL_MD 0xBE     //!< R   Calibration data (16 bits)

#define BMP085_CONTROL 0xF4         //!< Control register
#define BMP085_TEMPDATA 0xF6        //!< Temperature data register
#define BMP085_PRESSUREDATA 0xF6    //!< Pressure data register
#define BMP085_READTEMPCMD 0x2E     //!< Read temperature control register value
#define BMP085_READPRESSURECMD 0x34 //!< Read pressure control register value


BMP085::BMP085() : I2C() {
    _addr = BMP_ADDR;
}

bool BMP085::begin() {
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

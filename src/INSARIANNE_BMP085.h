/*


*/



#ifndef INSARIANNE_BMP085_H
#define INSARIANNE_BMP085_H

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
#define BMP_OK_REG 0xD0             //!< Read if the sensor is ready
#define BMP_OK 0x55                 //!< Sensor ready


class BMP085 : private I2C
{
  public:
    BMP085();

    bool begin(void);

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

#endif
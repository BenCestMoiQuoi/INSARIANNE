/*!
 * @file INSARIANNE.h
 *
 * This is a library for the INSARIANNE Project whitch is a base for the 
 * I2C and SPI devises
 *
 */

#ifndef INSARIANNE_H
#define INSARIANNE_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>


#define VERSION_LIB "1.0.0"


class I2C {
  protected:
    I2C(TwoWire *theWire = &Wire);
    virtual void begin(uint8_t address);

    bool write(uint8_t *data, size_t len, bool stop = true, \
               uint8_t *reg_data = nullptr, size_t reg_len = 0);
    uint8_t read8(uint8_t reg);
    uint16_t read16(uint8_t reg);
    bool read_n(uint8_t reg, uint8_t data[], int n);
    bool write_bits(uint8_t data, uint8_t reg, uint8_t bits, uint8_t shift);
    uint8_t read_bits(uint8_t reg, uint8_t bits, uint8_t shift);
  
    uint8_t _addr;
    TwoWire *_wire;
};

/*class SPI {

};
*/

#endif 

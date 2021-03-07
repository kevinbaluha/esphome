/***************************************************
  This is a library for the MLX90614 Temp Sensor

  Designed specifically to work with the MLX90614 sensors in the
  adafruit shop
  ----> https://www.adafruit.com/products/1748
  ----> https://www.adafruit.com/products/1749

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "MLX90614drv.h"

MLX90614drv::MLX90614drv(uint8_t i2caddr) { _addr = i2caddr; }

bool MLX90614drv::begin(void) {
  Wire.begin();
  return true;
}

uint16_t MLX90614drv::readEmissivityReg(void) {
  return read16(MLX90614_EMISS);
}

void MLX90614drv::writeEmissivityReg(uint16_t ereg) {
  write16(MLX90614_EMISS, 0); // erase
  delay(10);
  write16(MLX90614_EMISS, ereg);
  delay(10);
}

double MLX90614drv::readEmissivity(void) {
  uint16_t ereg = read16(MLX90614_EMISS);
  return ((double)ereg) / 65535.0;
}

void MLX90614drv::writeEmissivity(double emissivity) {
  uint16_t ereg = (uint16_t)(0xffff * emissivity);

  writeEmissivityReg(ereg);
}

double MLX90614drv::readAmbient(void) {
  return (readTemp(MLX90614_TA));
}

double MLX90614drv::readObject(void) {
  return (readTemp(MLX90614_TOBJ1));
}

double MLX90614drv::readTemp(uint8_t reg) {
  return (((double)read16(reg)) * 0.02);
}

uint16_t MLX90614drv::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(_addr); // start transmission to device
  Wire.write(a);                 // sends register address to read from
  Wire.endTransmission(false);   // end transmission

  Wire.requestFrom(_addr, (size_t)3); // send data n-bytes read
  ret = Wire.read();                  // receive DATA
  ret |= Wire.read() << 8;            // receive DATA

  uint8_t pec = Wire.read();

  return ret;
}

byte MLX90614drv::crc8(byte *addr, byte len)
// The PEC calculation includes all bits except the START, REPEATED START, STOP,
// ACK, and NACK bits. The PEC is a CRC-8 with polynomial X8+X2+X1+1.
{
  byte crc = 0;
  while (len--) {
    byte inbyte = *addr++;
    for (byte i = 8; i; i--) {
      byte carry = (crc ^ inbyte) & 0x80;
      crc <<= 1;
      if (carry)
        crc ^= 0x7;
      inbyte <<= 1;
    }
  }
  return crc;
}

void MLX90614drv::write16(uint8_t a, uint16_t v) {
  uint8_t pec;
  uint8_t pecbuf[4];

  pecbuf[0] = _addr << 1;
  pecbuf[1] = a;
  pecbuf[2] = v & 0xff;
  pecbuf[3] = v >> 8;
  pec = crc8(pecbuf, sizeof pecbuf);

  Wire.beginTransmission(_addr); // start transmission to device
  Wire.write(a);                 // sends register address to write
  Wire.write(v & 0xff);          // lo
  Wire.write(v >> 8);            // hi
  Wire.write(pec);               // pec
  Wire.endTransmission(true);    // end transmission
}

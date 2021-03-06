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

  Written by Limor Fried/Ladyada for Adafruied in any redistribution
 ****************************************************/

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace mlx90614 {

#define MLX90614_I2CADDR 0x5A

// RAM
#define MLX90614_RAWIR1 0x04
#define MLX90614_RAWIR2 0x05
#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MLX90614_TOBJ2 0x08
// EEPROM
#define MLX90614_TOMAX 0x20
#define MLX90614_TOMIN 0x21
#define MLX90614_PWMCTRL 0x22
#define MLX90614_TARANGE 0x23
#define MLX90614_EMISS 0x24
#define MLX90614_CONFIG 0x25
#define MLX90614_ADDR 0x2E
#define MLX90614_ID1 0x3C
#define MLX90614_ID2 0x3D
#define MLX90614_ID3 0x3E
#define MLX90614_ID4 0x3F

class mlx90614Component : public PollingComponent, public i2c::I2CDevice {
  public:
    void setup() override;

    void dump_config() override;

    void update() override;
    
    float get_setup_priority() const override;

    void set_ambient_temperature_sensor(sensor::Sensor *ambient_temperature_sensor) { ambient_temperature_sensor_ = ambient_temperature_sensor; }
    void set_object_temperature_sensor(sensor::Sensor *object_temperature_sensor) { object_temperature_sensor_ = object_temperature_sensor; }
    void set_emissivity_sensor(sensor::Sensor *emissivity_sensor) { emissivity_sensor_ = emissivity_sensor; }
    void set_emissivity(double emissivity) {emissivity_=clamp(emissivity, 0.1, 1.0); }
    uint16_t readAmbient(void) { return readTemp(MLX90614_TA); }
    uint16_t readObject(void) { return readTemp(MLX90614_TOBJ1); }
    uint16_t read16(uint8_t reg) {
      uint8_t d[3];

      uint8_t f = 0x80 |reg;
      raw_write(&reg, 1);

      //raw_request_from(3);
      read_bytes_raw(d, 3);
      raw_end_transmission(true);
      return (d[0] | d[1]<<8);
    }

    double readEmissivity(void) {
      return(static_cast<double>(read16(MLX90614_EMISS) / 65535.0));
    }

    void writeEmissivity(double emissivity) {
      uint16_t emissivity_v = static_cast<uint16_t>((emissivity * 65535.0) + .5);
      this->write_byte_16(MLX90614_EMISS, 0);
      this->write_byte_16(MLX90614_EMISS, emissivity_v);
    }

    float readTemp(uint8_t reg) {
      return (((float)read16(reg)));
    }

private:
  sensor::Sensor *ambient_temperature_sensor_{nullptr};
  sensor::Sensor *object_temperature_sensor_{nullptr};
  sensor::Sensor *emissivity_sensor_{nullptr};
  double emissivity_ = 1.0;
};
}
}

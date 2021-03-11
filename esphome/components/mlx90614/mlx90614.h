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
    void set_emissivity(double emissivity) {
      emissivity_=clamp(emissivity, 0.1, 1.0); 
      writeEmissivity();
    }
    float  readAmbient(void) { readTemp(MLX90614_TA); return readTemp(MLX90614_TA); }
    float  readObject(void) { readTemp(MLX90614_TOBJ1); return readTemp(MLX90614_TOBJ1); }

    double readEmissivity(void) {
      return(((double)read16(MLX90614_EMISS)) / 65535.0);
      return(((double)read16(MLX90614_EMISS)) / 65535.0);
    }

    void writeEmissivity() { writeEmissivity(emissivity_);}
    void writeEmissivity(double emissivity);

  private:
    double readTemp(uint8_t reg) { 
      return (((double)read16(reg))* 0.02); 
    }

    uint16_t read16(uint8_t reg);
    void write16(uint8_t reg, uint16_t data);
    byte crc8(byte *data, byte len);

    sensor::Sensor *ambient_temperature_sensor_{nullptr};
    sensor::Sensor *object_temperature_sensor_{nullptr};
    sensor::Sensor *emissivity_sensor_{nullptr};
    double emissivity_ = 0.5;
};
}
}

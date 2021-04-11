#ifndef __AHT20_H__
#define __AHT20_H__
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace aht20 {
    
class aht20Component : public PollingComponent, public i2c::I2CDevice {
  public:
    void setup() override;
    
    void dump_config() override;
    
    void update() override;

    void writeCmd(uint8_t command);
    uint8_t readReg(uint8_t reg);
    uint8_t getStatus();

    float get_setup_priority() const override;

    bool getSensor(float *h, float *t);
    bool getTemperature(float *t);
    bool getHumidity(float *h);

    void set_temperature_sensor(sensor::Sensor *t_sensor) { temperature_sensor_ = t_sensor; }
    void set_humidity_sensor(sensor::Sensor *h_sensor) { humidity_sensor_ = h_sensor ; }

private:
    uint8_t startSensor();
    sensor::Sensor *temperature_sensor_{nullptr};
    sensor::Sensor *humidity_sensor_{nullptr};

};
}
}

#endif

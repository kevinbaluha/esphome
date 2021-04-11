#ifndef TOF10120_H
#define TOF10120_H
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
namespace esphome {
namespace tof10120 {
#define TOF10120_READ_DISTANCE_CMD 0x00
#define TOF10120_DEFAULT_DELAY 30   
#define TOF10120_DIR_SEND_REGISTER 0x0e
#define TOF10120_DISTANCE_REGISTER 0x00
#define TOF10120_OUT_OF_RANGE_VALUE 2000

class tof10120Component : public PollingComponent, public i2c::I2CDevice {
  public:
    void setup() override;

    void dump_config() override;

    void update() override;

    float get_setup_priority() const override;

    double getDistance();
    void setRange(uint8_t range_mm);
    void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
    void set_max_range(uint8_t range_mm) { 
	max_range_mm_ = range_mm; setRange(range_mm);}
  protected:
    sensor::Sensor *distance_sensor_{nullptr};
    uint8_t range_mm_ = 0;
};
}
}

#endif // TOF10120_H

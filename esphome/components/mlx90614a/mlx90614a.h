#pragma once
#ifndef MLX90614A_H
#define MLX90614A_H
// some declarations in
// the header file.

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "MLX90614drv.h"

namespace esphome {
namespace mlx90614a {
static const char *TAG = "mlx90614";

class mlx90614aComponent : public PollingComponent {
  protected:
    MLX90614drv mlx = MLX90614drv();
  public:
    //mlx90614aComponent( uint32_t update_interval ) : PollingComponent(update_interval) { }
    void setup() override {
      mlx.begin();
    }

    void dump_config() override;

    void update() override;

    float get_setup_priority() const override {
        return setup_priority::DATA;
     }

    void set_ambient_temperature_sensor(sensor::Sensor *ambient_temperature_sensor) { ambient_temperature_sensor_ = ambient_temperature_sensor; }
    void set_object_temperature_sensor(sensor::Sensor *object_temperature_sensor) { object_temperature_sensor_ = object_temperature_sensor; }
    void set_emissivity_sensor(sensor::Sensor *emissivity_sensor) { emissivity_sensor_ = emissivity_sensor; }
    void set_emissivity(double emissivity) {
      emissivity_=clamp(emissivity, 0.1, 1); 
      mlx.writeEmissivity(emissivity_);
      mlx.writeEmissivity(emissivity_);
    }

    double readEmissivity(void) { mlx.readEmissivity(); return mlx.readEmissivity(); }
    double readAmbient(void) { mlx.readAmbient(); return mlx.readAmbient(); }
    double readObject(void) { mlx.readObject(); return mlx.readObject(); }
  private:
    sensor::Sensor *ambient_temperature_sensor_{nullptr};
    sensor::Sensor *object_temperature_sensor_{nullptr};
    sensor::Sensor *emissivity_sensor_{nullptr};
    double emissivity_ = 1.0;
};
}
}
#endif

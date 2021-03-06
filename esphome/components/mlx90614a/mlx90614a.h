#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
//#include "esphome/components/i2c/i2c.h"

#include "esphome.h"
//#include "Adafruit_MLX90614.h"

namespace esphome {
namespace mlx90614a {
static const char *TAG = "mlx90614";

class mlx90614aComponent : public PollingComponent {
  public:
    Adafruit_MLX90614 mlx = Adafruit_MLX90614();

    //mlx90614aComponent( uint32_t update_interval ) : PollingComponent(update_interval) { }
    void setup() override {
      mlx.begin();
    }

    void dump_config() override {
      ESP_LOGCONFIG(TAG, "mlx90614a:");
      LOG_UPDATE_INTERVAL(this);
      LOG_SENSOR("  ", "Ambient Temperature", this->ambient_temperature_sensor_);
      LOG_SENSOR("  ", "Object Temperature", this->object_temperature_sensor_);
    }

    void update() override {
      if (this->ambient_temperature_sensor_ != nullptr) {
        ambient_temperature_sensor_->publish_state(readAmbient());
      }

      if (this->emissivity_sensor_ != nullptr)
        this->emissivity_sensor_->publish_state(this->readEmissivity() * 100);

      if (this->object_temperature_sensor_ != nullptr) {
        object_temperature_sensor_->publish_state(readObject());
      }
    }

    float get_setup_priority() const override {
        return setup_priority::DATA;
     }

    void set_ambient_temperature_sensor(sensor::Sensor *ambient_temperature_sensor) { ambient_temperature_sensor_ = ambient_temperature_sensor; }
    void set_object_temperature_sensor(sensor::Sensor *object_temperature_sensor) { object_temperature_sensor_ = object_temperature_sensor; }
    void set_emissivity_sensor(sensor::Sensor *emissivity_sensor) { emissivity_sensor_ = emissivity_sensor; }
    void set_emissivity(double emissivity) {emissivity_=clamp(emissivity, 0.1, 1.0); }

    double readEmissivity(void) { mlx.readEmissivity(); return mlx.readEmissivity(); }
    double readAmbient(void) { mlx.readAmbientTempC(); return mlx.readAmbientTempC(); }
    double readObject(void) { mlx.readObjectTempC(); return mlx.readObjectTempC(); }
  private:
    sensor::Sensor *ambient_temperature_sensor_{nullptr};
    sensor::Sensor *object_temperature_sensor_{nullptr};
    sensor::Sensor *emissivity_sensor_{nullptr};
    double emissivity_ = 1.0;
};
}
}

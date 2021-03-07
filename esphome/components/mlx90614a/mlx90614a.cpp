#include "mlx90614a.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mlx90614a {

    void mlx90614aComponent::dump_config() {
      ESP_LOGCONFIG(TAG, "mlx90614a:");
      LOG_UPDATE_INTERVAL(this);
      LOG_SENSOR("  ", "Ambient Temperature", ambient_temperature_sensor_);
      LOG_SENSOR("  ", "Object Temperature", object_temperature_sensor_);
      LOG_SENSOR("  ", "Emissivity", emissivity_sensor_);
      ESP_LOGCONFIG(TAG, "%s%s:'%lf'", "  ", "initial_emissivity", emissivity_);
    }

    void mlx90614aComponent::update() {
      if (ambient_temperature_sensor_ != nullptr) {
        ambient_temperature_sensor_->publish_state(readAmbient());
      }

      if (emissivity_sensor_ != nullptr)
        emissivity_sensor_->publish_state(readEmissivity() * 100);

      if (object_temperature_sensor_ != nullptr) {
        object_temperature_sensor_->publish_state(readObject());
      }
    }
}
}

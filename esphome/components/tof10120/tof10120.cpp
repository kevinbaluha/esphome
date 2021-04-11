#include "tof10120.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tof10120 {

void tof10120Component::setup() {}

void tof10120Component::dump_config() {
  ESP_LOGCONFIG(TAG, "tof10120:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with tof10120 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);

}

void tof10120Component::update() {
  ESP_LOGV(TAG, "    Updating tof10120...");
  double distance = getDistance();
  if (this->distance_sensor_ != nullptr)
    this->distance_sensor_->publish_state(distance);
}

float tof10120Component::get_setup_priority() {
  return setup_priority::DATA;
}

double tof10120Component::getDistance() {
  uint8_t wb = TOF10120_READ_DISTANCE_CMD;
  if (!this->write_bytes(TOF10120_DISTANCE_REGISTER, &wb, sizeof(wb))) { 
    ESP_LOGE(TAG, "Communication with TOF10120 failed on write");
    this->status_set_warning();
    return (NAN);
  }

  uint8_t data[2];
  if (!this->read_bytes(TOF10120_DISTANCE_REGISTER, data, 2, TOF10120_DEFAULT_DELAY)) {
    ESP_LOGE(TAG, "Communication with TOF10120 failed on read");
    this->status_set_warning();
    return (NAN);
  }

  uint16_t distance_mm = (data[0] << 8) | data[1];
  ESP_LOGI(TAG, "Data read: %dmm", distance_mm);

  if (distance_mm == TOF10120_OUT_OF_RANGE_VALUE) {
    ESP_LOGW(TAG, "Distance measurement out of range");
    return(NAN);
  } else {
    return((double)distance_mm); 
  }
}
void setRange(uint8_t range_mm_max) {
  uint8_t wb = TOF10120_READ_DISTANCE_CMD;
  if (!this->write_bytes(TOF10120_DISTANCE_REGISTER, &wb, sizeof(wb))) { 
    ESP_LOGE(TAG, "Communication with TOF10120 failed on write");
    this->status_set_warning();
    return (NAN);
  }

  uint8_t data[2];
  if (!this->read_bytes(TOF10120_DISTANCE_REGISTER, data, 2, TOF10120_DEFAULT_DELAY)) {
    ESP_LOGE(TAG, "Communication with TOF10120 failed on read");
    this->status_set_warning();
    return (NAN);
  }

  uint16_t distance_mm = (data[0] << 8) | data[1];
  ESP_LOGI(TAG, "Data read: %dmm", distance_mm);

  if (distance_mm == TOF10120_OUT_OF_RANGE_VALUE) {
    ESP_LOGW(TAG, "Distance measurement out of range");
    return(NAN);
  } else {
    return((double)distance_mm); 
  }
}

}
}

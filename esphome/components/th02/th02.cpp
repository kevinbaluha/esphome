#include "th02.h"
#include "esphome/core/log.h"

namespace esphome {
namespace th02 {

static const char *TAG = "th02.sensor";

void TH02Component::setup() {
}
void TH02Component::dump_config() {
  ESP_LOGCONFIG(TAG, "TH02:");
  LOG_I2C_DEVICE(this);
  switch (this->error_code_) {
    case COMMUNICATION_FAILED:
      ESP_LOGE(TAG, "Communication with TH02 failed!");
      break;
    case WRONG_CHIP_ID:
      ESP_LOGE(TAG, "TH02 has wrong chip ID! Is it a TH02?");
      break;
    case NONE:
    default:
      break;
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}
float TH02Component::get_setup_priority() const { return setup_priority::DATA; }

void TH02Component::update() {
  // Enable sensor
  ESP_LOGV(TAG, "Sending conversion request...");
  uint8_t meas_register = 0;

  //this->set_timeout("data", uint32_t(ceilf(meas_time)), [this]() {
    float temperature = this->readTemp();
    if (isnan(temperature)) {
      ESP_LOGW(TAG, "Invalid temperature, cannot read pressure & humidity values.");
      this->status_set_warning();
      return;
    }
    float humidity = this->readHumidity();

    ESP_LOGD(TAG, "Got temperature=%.1f°C humidity=%.1f%%", temperature, humidity);
    if (this->temperature_sensor_ != nullptr)
      this->temperature_sensor_->publish_state(temperature);
    if (this->humidity_sensor_ != nullptr)
      this->humidity_sensor_->publish_state(humidity);
    //this->status_clear_warning();
  //});
}

double TH02Component::readTemp(void) {
    writeReg(TH02_REG_CONFIG, TH02_CMD_MEASURE_TEMP);
    while (!isAvailable());
    uint16_t value = readData();

    value = value >> 2;

    /* Formula: Temperature(C) = (Value/32) - 50 */

    double temper = (value / 32.0) - 50.0;

    return temper;
}

double TH02Component::readHumidity(void) {
    writeReg(TH02_REG_CONFIG, TH02_CMD_MEASURE_HUMI);

    while (!isAvailable());
    uint16_t value = readData();

    value = value >> 4;

    /* Formula: Humidity(%) = (Value/16) - 24 */

    double humidity = (value / 16.0) - 24.0;

    return humidity;
}

void TH02Component::writeCmd(uint8_t command) {
  raw_begin_transmission();
  raw_write(&command,1);
  raw_end_transmission(false);
}

uint8_t TH02Component::readReg(uint8_t reg) {
  uint8_t value = 0;
  writeCmd(reg);
  read_bytes_raw(&value, 1);
  return value;
}

void TH02Component::writeReg(uint8_t reg, uint8_t data) {
  raw_begin_transmission();
  raw_write(&reg,1);
  raw_write(&data,1);
  raw_end_transmission(true);
}

uint16_t TH02Component::readData2byte() {
    uint8_t regData[2] = {0};
    writeCmd(TH02_REG_DATA_H);
    read_bytes_raw(&regData[1], 1);
    writeCmd(TH02_REG_DATA_L);
    read_bytes_raw(&regData[0], 1);
    return (regData[1] << 8) | (regData[0]);
}


}  // namespace th02
}  // namespace esphome

#include "aht20.h"
#include "esphome/core/log.h"

namespace esphome {
namespace aht20 {

static const char *TAG = "aht20.sensor";


void aht20Component::setup()
{
    uint8_t reg = 0xbe;
    raw_begin_transmission();
    raw_write(&reg,1);
    raw_end_transmission();    // stop transmitting
}

uint8_t aht20Component::startSensor()
{
    uint8_t reg[3] = { 0xac, 0x33, 0x00 };
    raw_begin_transmission(); // transmit to device #8
    raw_write(reg,3);
    raw_end_transmission();    // stop transmitting

    uint8_t resp;
    read_bytes_raw(&resp, 1);
    return resp;
}

bool aht20Component::getSensor(float *h, float *t)
{
    startSensor();
    uint8_t data[6];
    read_bytes_raw(data, 6);

    if(data[0] & 0x80) return false;

    unsigned long __humi = 0;
    unsigned long __temp = 0;

    __humi = data[1];
    __humi <<= 8;
    __humi += data[2];
    __humi <<= 4;
    __humi += data[3] >> 4;

    *h = (float)__humi/1048576.0;

    __temp = data[3]&0x0f;
    __temp <<=8;
    __temp += data[4];
    __temp <<=8;
    __temp += data[5];

    *t = (float)__temp/1048576.0*200.0-50.0;

    return true;

}

bool aht20Component::getTemperature(float *t)
{
    float __t, __h;
    
    int ret = getSensor(&__h, &__t);
    if(0 == ret) return false;
    
    *t = __t;
    return true;
}

bool aht20Component::getHumidity(float *h)
{
    float __t, __h;
    
    int ret = getSensor(&__h, &__t);
    if(0 == ret) return false;
    
    *h = __h;
    return true;
}
float aht20Component::get_setup_priority() const { return setup_priority::DATA; }

void aht20Component::dump_config() {
  ESP_LOGCONFIG(TAG, "aht20:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with aht20 failed!");
  }
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}

void aht20Component::update() {
  float humidity = 0.0;
  float temperature = 0.0;

  if (!aht20Component::getSensor(&humidity, &temperature)) {
    return;
  }
  if (this->temperature_sensor_ != nullptr) {
    this->temperature_sensor_->publish_state(temperature);
  }
  if (this->humidity_sensor_ != nullptr) {
    this->humidity_sensor_->publish_state(humidity);
  }
}


}
}

#include "mlx90614.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mlx90614 {

static const char *TAG = "mlx90614";

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

void mlx90614Component::setup() {
  uint8_t raw_data[3];
  memset(&raw_data,0x00, sizeof(raw_data));
  this->read_bytes(MLX90614_ADDR, raw_data, 3);
  ESP_LOGVV("mlx90614", "%x %x %x ADDR", raw_data[0], raw_data[1], raw_data[2] );
}

void mlx90614Component::dump_config() {
  ESP_LOGCONFIG(TAG, "mlx90614:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with mlx90614 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Ambient Temperature", this->ambient_temperature_sensor_);
  LOG_SENSOR("  ", "Object Temperature", this->object_temperature_sensor_);
  LOG_SENSOR("  ", "Emissivity", this->emissivity_sensor_);
}

void mlx90614Component::update() {
  ESP_LOGV(TAG, "    Updating mlx90614...");
  double ambient = readAmbient();
  double object = readObject();
  if (this->ambient_temperature_sensor_ != nullptr)
    this->ambient_temperature_sensor_->publish_state(ambient);

  if (this->object_temperature_sensor_ != nullptr)
    this->object_temperature_sensor_->publish_state(object);

  if (this->emissivity_sensor_ != nullptr)
    this->emissivity_sensor_->publish_state(this->readEmissivity() * 100);
}

float mlx90614Component::get_setup_priority() const { 
	return setup_priority::DATA; 
}

uint16_t mlx90614Component::read16(uint8_t reg) {
  uint8_t d[3];
  raw_begin_transmission();
  raw_write(&reg, 1);
  raw_end_transmission(false);
  read_bytes_raw(d, 3);
  raw_end_transmission(true);
  return ( d[1]<<8 | d[0] );
}

void mlx90614Component::write16(uint8_t reg, uint16_t data) {
  uint8_t pec;
  uint8_t pecbuf[4];

  pecbuf[0] = address_ << 1;
  pecbuf[1] = reg;
  pecbuf[2] = data & 0xff;
  pecbuf[3] = data >> 8;
  pec = crc8(pecbuf, sizeof pecbuf);
  raw_begin_transmission();
  raw_write(&address_, 1);
  raw_write(&reg, 1);
  raw_write(&pecbuf[2],1);    // lo
  raw_write(&pecbuf[3],1);    // hi
  raw_write(&pec,1);          // pec
  raw_end_transmission(true);

}

byte mlx90614Component::crc8(byte *addr, byte len) {
  // The PEC calculation includes all bits except the START, REPEATED START, STOP,
  // ACK, and NACK bits. The PEC is a CRC-8 with polynomial X8+X2+X1+1.

  byte crc = 0;
  while (len--) {
    byte inbyte = *addr++;
    for (byte i = 8; i; i--) {
      byte carry = (crc ^ inbyte) & 0x80;
      crc <<= 1;
      if (carry)
	crc ^= 0x7;
      inbyte <<= 1;
    }
  }
  return crc;
}


}
}

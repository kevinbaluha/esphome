#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace th02 {

#define TH02_I2C_DEV_ID      	0x40
#define TH02_REG_STATUS         0x00
#define TH02_REG_DATA_H         0x01
#define TH02_REG_DATA_L         0x02
#define TH02_REG_CONFIG         0x03
#define TH02_REG_ID             0x11

#define TH02_STATUS_RDY_MASK    0x01    //poll RDY,0 indicate the conversion is done

#define TH02_CMD_MEASURE_HUMI   0x01    //perform a humidity measurement
#define TH02_CMD_MEASURE_TEMP   0x11    //perform a temperature measurement

#define TH02_WR_REG_MODE        0xC0
#define TH02_RD_REG_MODE        0x80


/// Internal struct storing the calibration values of an TH02.

/** Enum listing all Oversampling values for the TH02.
 *
 * Oversampling basically means measuring a condition multiple times. Higher oversampling
 * values therefore increase the time required to read sensor values but increase accuracy.
 */

/** Enum listing all Infinite Impulse Filter values for the TH02.
 *
 * Higher values increase accuracy, but decrease response time.
 */
class TH02Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_humidity_sensor(sensor::Sensor *humidity_sensor) { humidity_sensor_ = humidity_sensor; }

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

 protected:
  /// Read the temperature value and store the calculated ambient temperature in t_fine.
  public:
    uint8_t isAvailable() {
    	uint8_t status =  readReg(TH02_REG_STATUS);
    	return ((status & TH02_STATUS_RDY_MASK)?true:false);
    };
    double readTemp(void);
    double readHumidity(void);
  private:
    void     writeCmd(uint8_t u8Cmd);
    uint8_t  readReg(uint8_t u8Reg);
    void     writeReg(uint8_t u8Reg, uint8_t u8Data);
    uint16_t readData(void) { return readData2byte(); };
    uint16_t readData2byte(void);

    sensor::Sensor *temperature_sensor_;
    sensor::Sensor *humidity_sensor_;

    enum ErrorCode {
      NONE = 0,
      COMMUNICATION_FAILED,
      WRONG_CHIP_ID,
    } error_code_{NONE};
};

}  // namespace th02
}  // namespace esphome

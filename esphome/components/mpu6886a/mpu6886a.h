
/*
 Note: The MPU6886 is an I2C sensor and uses the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or
 a 3.3 V Teensy 3.1. We have disabled the internal pull-ups used by the Wire
 library in the Wire.h/twi.c utility file. We are also using the 400 kHz fast
 I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
#ifndef _MPU6886_H_
#define _MPU6886_H_

#pragma once
  
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace mpu6886a {

#define setbits2mask(val, mask, mask_flag)  ((val & ~mask) | (mask_flag?mask:0x00))
#define setbits2val(val, mask, mask_value)  ((val & ~mask) | (mask_value&mask))

#define MPU6886_ADDRESS           0x68 
#define MPU6886_WHOAMI            0x75
#define MPU6886_ACCEL_INTEL_CTRL  0x69
#define MPU6886_SMPLRT_DIV        0x19
#define MPU6886_INT_PIN_CFG       0x37
#define MPU6886_INT_ENABLE        0x38
#define MPU6886_FIFO_WM_INT_STATUS 0x39
#define MPU6886_INT_STATUS        0x3A
#define MPU6886_ACCEL_WOM_X_THR   0x20
#define MPU6886_ACCEL_WOM_Y_THR   0x21
#define MPU6886_ACCEL_WOM_Z_THR   0x22

#define MPU6886_ACCEL_XOUT_H      0x3B
#define MPU6886_ACCEL_XOUT_L      0x3C
#define MPU6886_ACCEL_YOUT_H      0x3D
#define MPU6886_ACCEL_YOUT_L      0x3E
#define MPU6886_ACCEL_ZOUT_H      0x3F
#define MPU6886_ACCEL_ZOUT_L      0x40

#define MPU6886_TEMP_OUT_H        0x41
#define MPU6886_TEMP_OUT_L        0x42

#define MPU6886_GYRO_XOUT_H       0x43
#define MPU6886_GYRO_XOUT_L       0x44
#define MPU6886_GYRO_YOUT_H       0x45
#define MPU6886_GYRO_YOUT_L       0x46
#define MPU6886_GYRO_ZOUT_H       0x47
#define MPU6886_GYRO_ZOUT_L       0x48

#define MPU6886_USER_CTRL         0x6A
#define MPU6886_PWR_MGMT_1        0x6B
#define MPU6886_PWR_MGMT_2        0x6C
#define MPU6886_CONFIG            0x1A
#define MPU6886_GYRO_CONFIG       0x1B
#define MPU6886_ACCEL_CONFIG      0x1C
#define MPU6886_ACCEL_CONFIG2     0x1D
#define MPU6886_FIFO_EN           0x23

//#define G (9.8)
#define RtA     57.324841
#define AtR    	0.0174533	
#define Gyro_Gr	0.0010653

#define X_INT_ENABLE		0x80
#define Y_INT_ENABLE		0x40
#define Z_INT_ENABLE		0x20
#define OVERFLOW_INT_ENABLE	0x10
#define GYRO_READY_INT		0x04
#define DATA_READY_INT		0x01
#define XYZ_INT_ENABLE	X_INT_ENABLE|Y_INT_ENABLE|Z_INT_ENABLE

#define ACCEL_CF_X_ST           0x80
#define ACCEL_CF_Y_ST           0x40
#define ACCEL_CF_Z_ST           0x20
#define ACCEL_CF_FS_SEL         0x0c
#define ACCEL_CF_LP_SAMP        0x30
#define ACCEL_CF_BYPASS_LP      0x08
#define ACCEL_CF_LP_FILTER      0x07

class MPU6886aComponent  : public PollingComponent, public i2c::I2CDevice {
  public:
    enum Ascale {
      AFS_2G = 0,
      AFS_4G,
      AFS_8G,
      AFS_16G
    };

    enum Gscale {
      GFS_250DPS = 0,
      GFS_500DPS,
      GFS_1000DPS,
      GFS_2000DPS
    };


    Gscale Gyscale = GFS_2000DPS;
    Ascale Acscale = AFS_8G;
  public:
    void Init(void);
    void enableWakeOnMotion(Ascale ascale, uint8_t thresh_num_lsb);
    void getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az);
    void getGyroAdc(int16_t* gx, int16_t* gy, int16_t* gz);
    void getTempAdc(int16_t *t);

    void getAccelData(float* ax, float* ay, float* az);
    void getGyroData(float* gx, float* gy, float* gz);
    void getTempData(float *t);

    void SetGyroFsr(Gscale scale);
    void SetAccelFsr(Ascale scale);

    void getAhrsData(float *pitch,float *roll,float *yaw);

    void SetINTPinActiveLogic(uint8_t level);
    void DisableAllIRQ();
    void ClearAllIRQ();

    void setup() override;
    void dump_config() override;

    void update() override;

    float get_setup_priority() const override;
    static void gpio_intr(MPU6886aComponent *arg);

    void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; }
    void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
    void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
    void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
    void set_interrupt_pin(GPIOPin *pin) { interrupt_pin_ = pin; }
    void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
    void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; }
    void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }

    void set_threshold(uint8_t threshold);
    void set_accel_x_self_test(uint8_t val);
    void set_accel_y_self_test(uint8_t val);
    void set_accel_z_self_test(uint8_t val);
    void set_accel_full_scale(uint8_t val);
    void set_accel_low_power_samples(uint8_t val);
    void set_accel_bybass_lp(uint8_t val);
    void set_accel_lp_filter(uint8_t val);
    void set_wo_move_x(uint8_t womX);
    void set_wo_move_y(uint8_t womY);
    void set_wo_move_z(uint8_t womZ);
    void set_wo_overflow(uint8_t wo_ov);
    void set_wo_gyro(uint8_t wo_gyro);
    void set_wo_data(uint8_t wo_data);
  protected:
    sensor::Sensor *accel_x_sensor_{nullptr};
    sensor::Sensor *accel_y_sensor_{nullptr};
    sensor::Sensor *accel_z_sensor_{nullptr};
    sensor::Sensor *temperature_sensor_{nullptr};
    sensor::Sensor *gyro_x_sensor_{nullptr};
    sensor::Sensor *gyro_y_sensor_{nullptr};
    sensor::Sensor *gyro_z_sensor_{nullptr};
    GPIOPin *interrupt_pin_{nullptr};
    uint8_t threshold_ = 0;
    uint8_t intr_wo_x = 0;
    uint8_t intr_wo_y = 0;
    uint8_t intr_wo_z = 0;
    uint8_t intr_wo_ov = 0;
    uint8_t intr_wo_gyro = 0;
    uint8_t intr_wo_data = 0;
    uint8_t intr_wake_flag = 0;
    uint8_t accel_config_[2] = {0,0};

  public:
    float aRes, gRes;

  private:
    void my_read_byte(uint8_t start_Addr, uint8_t *read_Buffer) { my_read_bytes( start_Addr, 1, read_Buffer); }
    void my_read_bytes(uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer);

    void my_write_byte(uint8_t start_Addr, uint8_t *write_Buffer) { my_write_bytes(start_Addr, 1, write_Buffer); }
    void my_write_bytes(uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer);
    void getGres();
    void getAres();

};
}
}
#endif


#include "mpu6886a.h"
#include "esphome/core/log.h"
#include "MahonyAHRS.h"
#include <math.h>

namespace esphome {
namespace mpu6886a {


static const char *TAG = "sensor.mpu6886a";

void MPU6886aComponent::update(void) {
  ESP_LOGV(TAG, "    Updating MPU6886...");

  float accel_x;
  float accel_y;
  float accel_z;
  getAccelData(&accel_x, &accel_y, &accel_z);

//  float temperature = data[3] / 340.0f + 36.53f;

  float gyro_x;
  float gyro_y;
  float gyro_z;
  getGyroData(&gyro_x, &gyro_y, &gyro_z);

  ESP_LOGD(TAG,
           "Got accel={x=%.3f m/s², y=%.3f m/s², z=%.3f m/s²}, "
           "gyro={x=%.3f °/s, y=%.3f °/s, z=%.3f °/s}, temp=%.3f°C",
           accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, 0.0 /* temperature */);

  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(accel_x);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(accel_y);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(accel_z);

  /* if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(temperature); */

  if (this->gyro_x_sensor_ != nullptr)
    this->gyro_x_sensor_->publish_state(gyro_x);
  if (this->gyro_y_sensor_ != nullptr)
    this->gyro_y_sensor_->publish_state(gyro_y);
  if (this->gyro_z_sensor_ != nullptr)
    this->gyro_z_sensor_->publish_state(gyro_z);

  this->status_clear_warning();
}

float MPU6886aComponent::get_setup_priority() const { return setup_priority::DATA; }

void MPU6886aComponent::setup(void) {
  Init();
  if (interrupt_pin_ != nullptr) {
    enableWakeOnMotion(AFS_16G,0x1f );
    interrupt_pin_->attach_interrupt(MPU6886aComponent::gpio_intr, this, CHANGE);

  }

}
void MPU6886aComponent::Init(void) {
  unsigned char tempdata[1];
  unsigned char regdata;
  
  read_byte(MPU6886_WHOAMI, tempdata);
  if(tempdata[0] != 0x19)
    this->mark_failed();
    return;
  
  regdata = 0x00;
  my_write_byte(MPU6886_PWR_MGMT_1, &regdata);

  regdata = (0x01<<7);
  my_write_byte(MPU6886_PWR_MGMT_1, &regdata);

  regdata = (0x01<<0);
  my_write_byte(MPU6886_PWR_MGMT_1, &regdata);

  regdata = 0x10;
  my_write_byte(MPU6886_ACCEL_CONFIG, &regdata);

  regdata = 0x18;
  my_write_byte(MPU6886_GYRO_CONFIG, &regdata);

  regdata = 0x01;
  my_write_byte(MPU6886_CONFIG, &regdata);

  regdata = 0x05;
  my_write_byte(MPU6886_SMPLRT_DIV, &regdata);

  regdata = 0x00;
  my_write_byte(MPU6886_INT_ENABLE, &regdata);

  regdata = 0x00;
  my_write_byte(MPU6886_ACCEL_CONFIG2, &regdata);

  regdata = 0x00;
  my_write_byte(MPU6886_USER_CTRL, &regdata);

  regdata = 0x00;
  my_write_byte(MPU6886_FIFO_EN, &regdata);

  if (interrupt_pin_ != nullptr) {
    regdata = interrupt_pin_->get_pin();
    ESP_LOGV(TAG, "Enabling Interrupts on pin %u", regdata);
    //regdata = 0x22;
    my_write_byte(MPU6886_INT_PIN_CFG, &regdata);

    regdata = 0x01;
    my_write_byte(MPU6886_INT_ENABLE, &regdata);
    SetINTPinActiveLogic(1);
    }

  getGres();
  getAres();
}

void MPU6886aComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "MPU6886:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MPU6886 failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  //LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_PIN("   Interrupt Pin", this->interrupt_pin_);
}

void MPU6886aComponent::my_read_bytes(uint8_t start_Addr, uint8_t length, uint8_t *read_Buffer) {
  raw_begin_transmission();
  raw_write(&start_Addr, 1);
  raw_end_transmission(false);
  read_bytes_raw(read_Buffer, length);
}

void MPU6886aComponent::my_write_bytes(uint8_t start_Addr, uint8_t length, uint8_t *write_Buffer) {

  raw_begin_transmission();
  raw_write(&start_Addr, 1);
  raw_write(write_Buffer, length);
  raw_end_transmission(true);

}

void MPU6886aComponent::enableWakeOnMotion(Ascale ascale, uint8_t thresh_num_lsb) {
    uint8_t regdata;
    /* 5.1 WAKE-ON-MOTION INTERRUPT
        The MPU-6886 provides motion detection capability. A qualifying motion sample is one where the high passed sample
        from any axis has an absolute value exceeding a user-programmable threshold. The following steps explain how to
        configure the Wake-on-Motion Interrupt.
    */

    /* Step 0: this isn't explicitly listed in the steps, but configuring the 
       FSR or full-scale-range of the accelerometer is important to setting up
       the accel/motion threshold in Step 4
    */
    regdata = (ascale << 3);
    my_write_byte(MPU6886_ACCEL_CONFIG, &regdata);

    /* Step 1: Ensure that Accelerometer is running
        • In PWR_MGMT_1 register (0x6B) set CYCLE = 0, SLEEP = 0, and GYRO_STANDBY = 0
        • In PWR_MGMT_2 register (0x6C) set STBY_XA = STBY_YA = STBY_ZA = 0, and STBY_XG = STBY_YG = STBY_ZG = 1
    */
    read_byte(MPU6886_PWR_MGMT_1, &regdata);
    regdata = regdata & 0b10001111; // set cyle, sleep, and gyro to standby, i.e. 0
    my_write_byte(MPU6886_PWR_MGMT_1, &regdata);

    regdata = 0b00000111; // set accel x, y, and z to standby 
    my_write_byte(MPU6886_PWR_MGMT_2, &regdata);

    /* Step 2: Set Accelerometer LPF bandwidth to 218.1 Hz
        • In ACCEL_CONFIG2 register (0x1D) set ACCEL_FCHOICE_B = 0 and A_DLPF_CFG[2:0] = 1 (b001)
    */
    read_byte(MPU6886_ACCEL_CONFIG2, &regdata);
    regdata = 0b00100001; // average 32 samples, use 218 Hz DLPF
    my_write_byte(MPU6886_ACCEL_CONFIG2, &regdata);

    /* Step 2.5 - active low? */
    read_byte(MPU6886_INT_PIN_CFG, &regdata);
    regdata =  ((regdata | 0b10000000) & 0b11011111); // configure pin active-low, no latch
    my_write_byte(MPU6886_INT_PIN_CFG, &regdata);

    /* Step 3: Enable Motion Interrupt
        • In INT_ENABLE register (0x38) set WOM_INT_EN = 111 to enable motion interrupt
    */
    regdata = XYZ_INT_ENABLE;// enable wake-on-motion interrupt for X, Y, and Z axes
    my_write_byte(MPU6886_INT_ENABLE, &regdata);
    
    /* Step 4: Set Motion Threshold
        • Set the motion threshold in ACCEL_WOM_THR register (0x1F)
        NOTE: the data sheet mentions 0x1F, but is probably referring to
              registers 0x20, 0x21, and 0x22 based on empirical tests
    */
    regdata = thresh_num_lsb; // set accel motion threshold for X, Y, and Z axes
    my_write_byte(MPU6886_ACCEL_WOM_X_THR, &regdata);
    my_write_byte(MPU6886_ACCEL_WOM_Y_THR, &regdata);
    my_write_byte(MPU6886_ACCEL_WOM_Z_THR, &regdata);

    /* Step 5: Enable Accelerometer Hardware Intelligence
        • In ACCEL_INTEL_CTRL register (0x69) set ACCEL_INTEL_EN = ACCEL_INTEL_MODE = 1;
          Ensure that bit 0 is set to 0
    */
    regdata = 0b11000010; // enable wake-on-motion if any of X, Y, or Z axes is above threshold
    // WOM_STEP5_ACCEL_INTEL_CTRL_INTEL_EN_1_MODE_1_WOM_TH_MODE_0;
    my_write_byte(MPU6886_ACCEL_INTEL_CTRL, &regdata);

    /* Step 7: Set Frequency of Wake-Up
        • In SMPLRT_DIV register (0x19) set SMPLRT_DIV[7:0] = 3.9 Hz – 500 Hz
    */
    // sample_rate = 1e3 / (1 + regdata)
    //   4.0 Hz = 1e3 / (1 + 249)
    //  10.0 Hz = 1e3 / (1 +  99)
    //  20.0 Hz = 1e3 / (1 +  49)
    //  25.0 Hz = 1e3 / (1 +  39)
    //  50.0 Hz = 1e3 / (1 +  19) <----
    // 500.0 Hz = 1e3 / (1 +   1)
    regdata = 19;
    my_write_byte(MPU6886_SMPLRT_DIV, &regdata);

    /* Step 8: Enable Cycle Mode (Accelerometer Low-Power Mode)
        • In PWR_MGMT_1 register (0x6B) set CYCLE = 1
    */
    read_byte(MPU6886_PWR_MGMT_1, &regdata);
    regdata = regdata | 0b00100000; // enable accelerometer low-power mode
    my_write_byte(MPU6886_PWR_MGMT_1, &regdata);
}

void MPU6886aComponent::getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az) {

   uint8_t buf[6];  
   my_read_bytes(MPU6886_ACCEL_XOUT_H,6,buf);
   
   *ax=((int16_t)buf[0]<<8)|buf[1];
   *ay=((int16_t)buf[2]<<8)|buf[3];
   *az=((int16_t)buf[4]<<8)|buf[5];

}
void MPU6886aComponent::getGyroAdc(int16_t* gx, int16_t* gy, int16_t* gz) {

  uint8_t buf[6];
  my_read_bytes(MPU6886_GYRO_XOUT_H,6,buf);
  
  *gx=((uint16_t)buf[0]<<8)|buf[1];  
  *gy=((uint16_t)buf[2]<<8)|buf[3];  
  *gz=((uint16_t)buf[4]<<8)|buf[5];
  
}

void MPU6886aComponent::getTempAdc(int16_t *t) {
  
  uint8_t buf[2];  
  my_read_bytes(MPU6886_TEMP_OUT_H,2,buf);
  
  *t=((uint16_t)buf[0]<<8)|buf[1];  
}


//!俯仰，航向，横滚：pitch，yaw，roll，指三维空间中飞行器的旋转状态。
void MPU6886aComponent::getAhrsData(float *pitch,float *roll,float *yaw) {

  float accX = 0; 
  float accY = 0;
  float accZ = 0;

  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;

  getGyroData(&gyroX,&gyroY,&gyroZ);
  getAccelData(&accX,&accY,&accZ);
  
  MahonyAHRSupdateIMU(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ,pitch,roll,yaw);

}

void MPU6886aComponent::getGres() {

   switch (Gyscale)
   {
  // Possible gyro scales (and their register bit settings) are:
     case GFS_250DPS:
           gRes = 250.0/32768.0;
           break;
     case GFS_500DPS:
           gRes = 500.0/32768.0;
           break;
     case GFS_1000DPS:
           gRes = 1000.0/32768.0;
           break;
     case GFS_2000DPS:
           gRes = 2000.0/32768.0;
           break;
   }

}


void MPU6886aComponent::getAres() {
   switch (Acscale)
   {
   // Possible accelerometer scales (and their register bit settings) are:
   // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
   // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }

}
 
void MPU6886aComponent::SetGyroFsr(Gscale scale)
{
    unsigned char regdata;	
    regdata = (scale<<3);
    my_write_byte(MPU6886_GYRO_CONFIG, &regdata);

    Gyscale = scale;
    getGres();
}

void MPU6886aComponent::SetAccelFsr(Ascale scale)
{
    unsigned char regdata;	
    regdata = (scale<<3);
    my_write_byte(MPU6886_ACCEL_CONFIG, &regdata);

    Acscale = scale;
    getAres();
}

void MPU6886aComponent::getAccelData(float* ax, float* ay, float* az) {
  int16_t accX = 0;
  int16_t accY = 0;
  int16_t accZ = 0;
  getAccelAdc(&accX,&accY,&accZ);


  *ax = (float)accX * aRes;
  *ay = (float)accY * aRes;
  *az = (float)accZ * aRes;

}
      
void MPU6886aComponent::getGyroData(float* gx, float* gy, float* gz) {
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
  getGyroAdc(&gyroX,&gyroY,&gyroZ);

  *gx = (float)gyroX * gRes;
  *gy = (float)gyroY * gRes;
  *gz = (float)gyroZ * gRes;
}

void MPU6886aComponent::getTempData(float *t) {
  
  int16_t temp = 0;
  getTempAdc(&temp);
  
  *t = (float)temp / 326.8 + 25.0;
}

void MPU6886aComponent::SetINTPinActiveLogic(uint8_t level) {
  uint8_t tempdata;
  read_byte(MPU6886_INT_PIN_CFG, &tempdata);
  tempdata &= 0x7f;
  tempdata |= level ? 0x00 : (0x01 << 7);
  my_write_byte(MPU6886_INT_PIN_CFG, &tempdata);
}

void MPU6886aComponent::DisableAllIRQ() {
  uint8_t tempdata = 0x00;
  my_write_byte(MPU6886_INT_ENABLE, &tempdata);
  read_byte(MPU6886_INT_PIN_CFG, &tempdata);
  tempdata |= 0x01 << 6;
  // int pin is configured as open drain
  my_write_byte(MPU6886_INT_PIN_CFG, &tempdata);
}

void MPU6886aComponent::ClearAllIRQ() {
  uint8_t tempdata = 0x00;
  read_byte(MPU6886_FIFO_WM_INT_STATUS, &tempdata);
  read_byte(MPU6886_INT_STATUS, &tempdata);
}

void ICACHE_RAM_ATTR MPU6886aComponent::gpio_intr(MPU6886aComponent *arg) {
  arg->update();
  arg->ClearAllIRQ();
}


}
}

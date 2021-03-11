import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c, sensor
from esphome.const import CONF_ID, CONF_TEMPERATURE, DEVICE_CLASS_EMPTY, DEVICE_CLASS_TEMPERATURE, \
    ICON_BRIEFCASE_DOWNLOAD, ICON_EMPTY, UNIT_METER_PER_SECOND_SQUARED, \
    ICON_SCREEN_ROTATION, UNIT_DEGREE_PER_SECOND, UNIT_CELSIUS

DEPENDENCIES = ['i2c']

CONF_ACCEL_X = 'accel_x'
CONF_ACCEL_Y = 'accel_y'
CONF_ACCEL_Z = 'accel_z'
CONF_GYRO_X = 'gyro_x'
CONF_GYRO_Y = 'gyro_y'
CONF_GYRO_Z = 'gyro_z'
CONF_INTERRUPT = 'interrupt'
CONF_INTERRUPT_PIN = 'pin'
CONF_INTERRUPT_THRESHOLD = 'threshold'
CONF_INTERRUPT_WOM_X = 'wake_on_x'
CONF_INTERRUPT_WOM_Y = 'wake_on_y'
CONF_INTERRUPT_WOM_Z = 'wake_on_z'
CONF_INTERRUPT_WO_OV = 'wake_on_overflow'
CONF_INTERRUPT_WO_GYRO = 'wake_on_gyro'
CONF_INTERRUPT_WO_DATA = 'wake_on_data'
CONF_ACCEL = 'accelerometer'
CONF_GYRO = 'gyroscope'
CONF_SELF_TEST = 'self_test'
CONF_FULL_SCALE = 'full_scale'
CONF_LOW_POWER_SAMPLES = 'low_power_samples'
CONF_BYPASS_LOW_POWER = 'low_power_bypass'
CONF_LOW_POWER_FILTER = 'low_power_filter'

mpu6886a_ns = cg.esphome_ns.namespace('mpu6886a')
MPU6886aComponent = mpu6886a_ns.class_('MPU6886aComponent', cg.PollingComponent, i2c.I2CDevice)

s_t = cv.Schema({
    cv.Optional(CONF_SELF_TEST): cv.boolean,
})

more_accel = cv.Schema({
})

accel_schema = sensor.sensor_schema(UNIT_METER_PER_SECOND_SQUARED, ICON_BRIEFCASE_DOWNLOAD, 2, DEVICE_CLASS_EMPTY).extend(s_t)
gyro_schema = sensor.sensor_schema(UNIT_DEGREE_PER_SECOND, ICON_SCREEN_ROTATION, 2,
                                   DEVICE_CLASS_EMPTY)
temperature_schema = sensor.sensor_schema(UNIT_CELSIUS, ICON_EMPTY, 1, DEVICE_CLASS_TEMPERATURE)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MPU6886aComponent),
    cv.Optional(CONF_ACCEL): cv.Schema({
       cv.Optional(CONF_ACCEL_X): accel_schema,
       cv.Optional(CONF_ACCEL_Y): accel_schema,
       cv.Optional(CONF_ACCEL_Z): accel_schema,
       cv.Optional(CONF_FULL_SCALE): cv.int_,
       cv.Optional(CONF_LOW_POWER_SAMPLES): cv.int_,
       cv.Optional(CONF_BYPASS_LOW_POWER): cv.boolean,
       cv.Optional(CONF_LOW_POWER_FILTER): cv.int_ }),
    cv.Optional(CONF_GYRO): cv.Schema({
       cv.Optional(CONF_GYRO_X): gyro_schema,
       cv.Optional(CONF_GYRO_Y): gyro_schema,
       cv.Optional(CONF_GYRO_Z): gyro_schema }),
    cv.Optional(CONF_TEMPERATURE): temperature_schema,
    cv.Optional(CONF_INTERRUPT): cv.Schema({ 
    	cv.Optional(CONF_INTERRUPT_PIN): cv.All(pins.internal_gpio_input_pin_schema,
                                       pins.validate_has_interrupt),
        cv.Optional(CONF_INTERRUPT_THRESHOLD): cv.int_,
        cv.Optional(CONF_INTERRUPT_WOM_X, default=True): cv.boolean,
        cv.Optional(CONF_INTERRUPT_WOM_Y, default=True): cv.boolean,
        cv.Optional(CONF_INTERRUPT_WOM_Z, default=True): cv.boolean,
        cv.Optional(CONF_INTERRUPT_WO_OV, default=False): cv.boolean,
        cv.Optional(CONF_INTERRUPT_WO_GYRO, default=False): cv.boolean,
        cv.Optional(CONF_INTERRUPT_WO_DATA, default=True): cv.boolean,
        }),
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x68))


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)
    

    if CONF_ACCEL in config:  
        if CONF_FULL_SCALE in config[CONF_ACCEL]:
            cg.add(var.set_accel_full_scale(config[CONF_ACCEL][CONF_FULL_SCALE]))
        if CONF_LOW_POWER_SAMPLES in config[CONF_ACCEL]:
            cg.add(var.set_accel_low_power_samples(config[CONF_ACCEL][CONF_LOW_POWER_SAMPLES]))
        if CONF_BYPASS_LOW_POWER in config[CONF_ACCEL]:
            cg.add(var.set_accel_low_power_samples(config[CONF_ACCEL][CONF_BYPASS_LOW_POWER]))
        if CONF_LOW_POWER_FILTER in config[CONF_ACCEL]:
            cg.add(var.set_accel_lp_filter(config[CONF_ACCEL][CONF_LOW_POWER_FILTER]))
  
    for d in ['x', 'y', 'z']:
        accel_key = f'accel_{d}'
        if CONF_ACCEL in config and accel_key in config[CONF_ACCEL]:
            sens = yield sensor.new_sensor(config[CONF_ACCEL][accel_key])
            cg.add(getattr(var, f'set_accel_{d}_sensor')(sens))
            if CONF_SELF_TEST in config[CONF_ACCEL][accel_key]:
              cg.add(getattr(var, f'set_accel_{d}_self_test')(config[CONF_ACCEL][accel_key][CONF_SELF_TEST]))
        accel_key = f'gyro_{d}'
        if CONF_GYRO in config and accel_key in config[CONF_GYRO]:
            sens = yield sensor.new_sensor(config[CONF_GYRO][accel_key])
            cg.add(getattr(var, f'set_gyro_{d}_sensor')(sens))

    if CONF_TEMPERATURE in config:
        sens = yield sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_INTERRUPT in config:
        if CONF_INTERRUPT_PIN in config[CONF_INTERRUPT]:
           pin = yield cg.gpio_pin_expression(config[CONF_INTERRUPT][CONF_INTERRUPT_PIN])
           cg.add(var.set_interrupt_pin(pin))

        if CONF_INTERRUPT_THRESHOLD in config[CONF_INTERRUPT]:
           cg.add(var.set_threshold(config[CONF_INTERRUPT][CONF_INTERRUPT_THRESHOLD]))

        if CONF_INTERRUPT_WOM_X in config[CONF_INTERRUPT]:
           cg.add(var.set_wo_move_x(config[CONF_INTERRUPT][CONF_INTERRUPT_WOM_X]))

        if CONF_INTERRUPT_WOM_Y in config[CONF_INTERRUPT]:
           cg.add(var.set_wo_move_y(config[CONF_INTERRUPT][CONF_INTERRUPT_WOM_Y]))

        if CONF_INTERRUPT_WOM_Z in config[CONF_INTERRUPT]:
           cg.add(var.set_wo_move_z(config[CONF_INTERRUPT][CONF_INTERRUPT_WOM_Z]))

        if CONF_INTERRUPT_WO_OV in config[CONF_INTERRUPT]:
           cg.add(var.set_wo_overflow(config[CONF_INTERRUPT][CONF_INTERRUPT_WO_OV]))

        if CONF_INTERRUPT_WO_GYRO in config[CONF_INTERRUPT]:
           cg.add(var.set_wo_gyro(config[CONF_INTERRUPT][CONF_INTERRUPT_WO_GYRO]))

        if CONF_INTERRUPT_WO_DATA in config[CONF_INTERRUPT]:
           cg.add(var.set_wo_data(config[CONF_INTERRUPT][CONF_INTERRUPT_WO_DATA]))




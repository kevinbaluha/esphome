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
CONF_INTERRUPT_PIN = 'interrupt_pin'

mpu6886_ns = cg.esphome_ns.namespace('mpu6886')
MPU6886Component = mpu6886_ns.class_('MPU6886Component', cg.PollingComponent, i2c.I2CDevice)

accel_schema = sensor.sensor_schema(UNIT_METER_PER_SECOND_SQUARED, ICON_BRIEFCASE_DOWNLOAD, 2,
                                    DEVICE_CLASS_EMPTY)
gyro_schema = sensor.sensor_schema(UNIT_DEGREE_PER_SECOND, ICON_SCREEN_ROTATION, 2,
                                   DEVICE_CLASS_EMPTY)
temperature_schema = sensor.sensor_schema(UNIT_CELSIUS, ICON_EMPTY, 1, DEVICE_CLASS_TEMPERATURE)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MPU6886Component),
    cv.Optional(CONF_ACCEL_X): accel_schema,
    cv.Optional(CONF_ACCEL_Y): accel_schema,
    cv.Optional(CONF_ACCEL_Z): accel_schema,
    cv.Optional(CONF_GYRO_X): gyro_schema,
    cv.Optional(CONF_GYRO_Y): gyro_schema,
    cv.Optional(CONF_GYRO_Z): gyro_schema,
    cv.Optional(CONF_TEMPERATURE): temperature_schema,
    cv.Optional(CONF_INTERRUPT_PIN): cv.All(pins.internal_gpio_input_pin_schema,
                                       pins.validate_has_interrupt),
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x68))


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)

    for d in ['x', 'y', 'z']:
        accel_key = f'accel_{d}'
        if accel_key in config:
            sens = yield sensor.new_sensor(config[accel_key])
            cg.add(getattr(var, f'set_accel_{d}_sensor')(sens))
        accel_key = f'gyro_{d}'
        if accel_key in config:
            sens = yield sensor.new_sensor(config[accel_key])
            cg.add(getattr(var, f'set_gyro_{d}_sensor')(sens))

    if CONF_TEMPERATURE in config:
        sens = yield sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_INTERRUPT_PIN in config:
        sens = yield sensor.new_sensor(config[CONF_INTERRUPT_PIN])
        cg.add(var.set_interupt_pin(sens))


import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import CONF_HUMIDITY, CONF_ID, CONF_IIR_FILTER, CONF_OVERSAMPLING, \
    CONF_PRESSURE, CONF_TEMPERATURE, DEVICE_CLASS_HUMIDITY, DEVICE_CLASS_PRESSURE, \
    DEVICE_CLASS_TEMPERATURE, ICON_EMPTY, UNIT_CELSIUS, UNIT_HECTOPASCAL, UNIT_PERCENT

DEPENDENCIES = ['i2c']

th02_ns = cg.esphome_ns.namespace('th02')

TH02Component = th02_ns.class_('TH02Component', cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(TH02Component),
    cv.Optional(CONF_TEMPERATURE):
        sensor.sensor_schema(UNIT_CELSIUS, ICON_EMPTY, 4, DEVICE_CLASS_TEMPERATURE),
    cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(UNIT_PERCENT, ICON_EMPTY, 6, DEVICE_CLASS_HUMIDITY),
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x40))


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)

    if CONF_TEMPERATURE in config:
        sens = yield sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_HUMIDITY in config:
        sens = yield sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(var.set_humidity_sensor(sens))


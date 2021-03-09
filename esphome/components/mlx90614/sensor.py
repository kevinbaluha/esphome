import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c, sensor
from esphome.const import CONF_ID, CONF_TEMPERATURE, DEVICE_CLASS_EMPTY, DEVICE_CLASS_TEMPERATURE, \
    ICON_BRIEFCASE_DOWNLOAD, ICON_THERMOMETER, ICON_EMPTY, UNIT_METER_PER_SECOND_SQUARED, \
    ICON_SCREEN_ROTATION, UNIT_DEGREE_PER_SECOND, UNIT_KELVIN, UNIT_PERCENT

DEPENDENCIES = ['i2c']

CONF_AMBIENT_TEMP = 'ambient_temperature'
CONF_OBJECT_TEMP = 'object_temperature'
CONF_EMISSIVITY = 'emissivity'
CONF_INITIAL_EMISSIVITY = 'initial_emissivity'

mlx90614_ns = cg.esphome_ns.namespace('mlx90614')
mlx90614Component = mlx90614_ns.class_('mlx90614Component', cg.PollingComponent, i2c.I2CDevice)

temperature_schema = sensor.sensor_schema(UNIT_KELVIN, ICON_THERMOMETER, 1, DEVICE_CLASS_TEMPERATURE)
emissivity_schema = sensor.sensor_schema(UNIT_PERCENT, ICON_THERMOMETER, 0, DEVICE_CLASS_EMPTY)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(mlx90614Component),
    cv.Required(CONF_OBJECT_TEMP): temperature_schema,
    cv.Optional(CONF_AMBIENT_TEMP): temperature_schema,
    cv.Optional(CONF_INITIAL_EMISSIVITY): cv.percentage,
    cv.Optional(CONF_EMISSIVITY): emissivity_schema 
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x5a))


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)

    if CONF_OBJECT_TEMP in config:
        sens = yield sensor.new_sensor(config[CONF_OBJECT_TEMP])
        cg.add(var.set_object_temperature_sensor(sens))

    if CONF_AMBIENT_TEMP in config:
        sens = yield sensor.new_sensor(config[CONF_AMBIENT_TEMP])
        cg.add(var.set_ambient_temperature_sensor(sens))

    if CONF_EMISSIVITY in config:
        sens = yield sensor.new_sensor(config[CONF_EMISSIVITY])
        cg.add(var.set_emissivity_sensor(sens))

    if CONF_INITIAL_EMISSIVITY in config:
        cg.add(var.set_emissivity(config[CONF_INITIAL_EMISSIVITY]))


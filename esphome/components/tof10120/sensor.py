import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c, sensor
from esphome.const import CONF_ID, DEVICE_CLASS_EMPTY, \
    ICON_EMPTY, ICON_SCREEN_ROTATION, UNIT_MILLIMETER

DEPENDENCIES = ['i2c']

CONF_DISTANCE = 'distance'

tof10120_ns = cg.esphome_ns.namespace('tof10120')
tof10120Component = tof10120_ns.class_('tof10120Component', cg.PollingComponent, i2c.I2CDevice)

distance_schema = sensor.sensor_schema(UNIT_MILLIMETER, ICON_EMPTY, 2, DEVICE_CLASS_EMPTY)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(tof10120Component),
    cv.Required(CONF_DISTANCE): distance_schema,
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x52))

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)

    if CONF_DISTANCE in config:
        sens = yield sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))

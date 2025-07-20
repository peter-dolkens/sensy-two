import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, text_sensor
from esphome.const import CONF_ID, CONF_UART_ID

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

sensytwo_ns = cg.esphome_ns.namespace("sensytwo")
SensyTwoComponent = sensytwo_ns.class_("SensyTwoComponent", cg.Component, uart.UARTDevice)


SENSOR_KEYS = [
    "t1_x",
    "t1_y",
    "t1_angle",
    "t1_speed",
    "t1_distance_resolution",
    "t1_distance",
    "t2_x",
    "t2_y",
    "t2_angle",
    "t2_speed",
    "t2_distance_resolution",
    "t2_distance",
    "t3_x",
    "t3_y",
    "t3_angle",
    "t3_speed",
    "t3_distance_resolution",
    "t3_distance",
]

TEXT_SENSOR_KEYS = [
    "radar_firmware",
    "radar_mac",
]

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SensyTwoComponent),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        }
    )
    .extend({cv.Optional(key): sensor.sensor_schema() for key in SENSOR_KEYS})
    .extend({cv.Optional(key): text_sensor.text_sensor_schema() for key in TEXT_SENSOR_KEYS})
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    uart_var = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], uart_var)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    for key in SENSOR_KEYS:
        if key in config:
            await sensor.register_sensor(getattr(var, key), config[key])

    for key in TEXT_SENSOR_KEYS:
        if key in config:
            await text_sensor.register_text_sensor(getattr(var, key), config[key])

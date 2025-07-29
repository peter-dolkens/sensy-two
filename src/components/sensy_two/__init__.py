import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, text_sensor
from esphome.const import CONF_ID, CONF_UART_ID

CONF_USE_PERSON_FRAMES = "use_person_frames"

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

sensytwo_ns = cg.esphome_ns.namespace("sensytwo")
SensyTwoComponent = sensytwo_ns.class_("SensyTwoComponent", cg.Component, uart.UARTDevice)


FIELDS = [
    "x",
    "y",
    "z",
    "angle",
    "speed",
    "distance_resolution",
    "distance",
]

SENSOR_KEYS = [
    f"t{i}_{field}"
    for i in range(1, 11)
    for field in FIELDS
]

TEXT_SENSOR_KEYS = [
    "radar_firmware",
    "radar_mac",
    "sensy_firmware",
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
    .extend({cv.Optional(CONF_USE_PERSON_FRAMES, default=True): cv.boolean})
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

    if CONF_USE_PERSON_FRAMES in config:
        cg.add(var.set_use_person_frames(config[CONF_USE_PERSON_FRAMES]))

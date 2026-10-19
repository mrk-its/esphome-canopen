"""`text_sensor: - platform: canopen`: cover / alarm state or on / off of a remote node's entity."""

import esphome.config_validation as cv
from esphome.components import text_sensor

from . import REMOTE_STATE_SCHEMA, attach_remote_state, layout, ns, validate_remote_state

DEPENDENCIES = ["canopen"]

CanopenRemoteTextSensor = ns.class_("CanopenRemoteTextSensor", text_sensor.TextSensor)

CONFIG_SCHEMA = cv.All(
    text_sensor.text_sensor_schema(CanopenRemoteTextSensor).extend(
        REMOTE_STATE_SCHEMA
    ),
    validate_remote_state(
        "text_sensor",
        [layout.KIND_BOOL, layout.KIND_COVER_STATE, layout.KIND_ALARM_STATE],
    ),
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    await attach_remote_state(var, config)

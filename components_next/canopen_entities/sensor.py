"""`sensor: - platform: canopen_entities`: state of a remote node's entity, see remote_state_slots().

Booleans are 0 / 1, percentages 0..1, color temperatures in mireds, cover / alarm
states their codes (OBJECT_DICTIONARY.md), sensor / number values decoded.
"""

from esphome.components import sensor
import esphome.config_validation as cv
from esphome.types import ConfigType

from . import (
    REMOTE_STATE_SCHEMA,
    RemoteSensor,
    attach_remote_state,
    layout,
    validate_remote_state,
)

DEPENDENCIES = ["canopen_entities"]

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(RemoteSensor).extend(REMOTE_STATE_SCHEMA),
    validate_remote_state("sensor", list(layout.KIND_CODES)),
)


async def to_code(config: ConfigType) -> None:
    var = await sensor.new_sensor(config)
    await attach_remote_state(var, config)

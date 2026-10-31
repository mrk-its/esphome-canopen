"""`binary_sensor: - platform: canopen_entity_mapper`: on / off state of a remote node's entity."""

from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.types import ConfigType

from . import (
    REMOTE_STATE_SCHEMA,
    RemoteBinarySensor,
    attach_remote_state,
    layout,
    validate_remote_state,
)

DEPENDENCIES = ["canopen_entity_mapper"]

CONFIG_SCHEMA = cv.All(
    binary_sensor.binary_sensor_schema(RemoteBinarySensor).extend(REMOTE_STATE_SCHEMA),
    validate_remote_state("binary_sensor", [layout.KIND_BOOL]),
)


async def to_code(config: ConfigType) -> None:
    var = await binary_sensor.new_binary_sensor(config)
    await attach_remote_state(var, config)

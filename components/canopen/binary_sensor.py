"""`binary_sensor: - platform: canopen`: state of a remote node's entity, see remote_state_slots()."""

import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import REMOTE_STATE_SCHEMA, attach_remote_state, layout, ns, validate_remote_state

DEPENDENCIES = ["canopen"]

CanopenRemoteBinarySensor = ns.class_(
    "CanopenRemoteBinarySensor", binary_sensor.BinarySensor
)

CONFIG_SCHEMA = cv.All(
    binary_sensor.binary_sensor_schema(CanopenRemoteBinarySensor).extend(
        REMOTE_STATE_SCHEMA
    ),
    validate_remote_state("binary_sensor", [layout.KIND_BOOL]),
)


async def to_code(config):
    var = await binary_sensor.new_binary_sensor(config)
    await attach_remote_state(var, config)

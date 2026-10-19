"""`sensor: - platform: canopen`: state of a remote node's entity, see remote_state_slots().

Booleans are 0 / 1, percentages 0..1, color temperatures in mireds, cover / alarm
states their codes (OBJECT_DICTIONARY.md), sensor / number values decoded.
"""

import esphome.config_validation as cv
from esphome.components import sensor

from . import REMOTE_STATE_SCHEMA, attach_remote_state, layout, ns, validate_remote_state

DEPENDENCIES = ["canopen"]

CanopenRemoteSensor = ns.class_("CanopenRemoteSensor", sensor.Sensor)

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(CanopenRemoteSensor).extend(REMOTE_STATE_SCHEMA),
    validate_remote_state("sensor", list(layout.KIND_CODES)),
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await attach_remote_state(var, config)

"""Hardware test of the canopen profile API (CANopenNode base class, register_node())."""

import esphome.codegen as cg
from esphome.components import canopen
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["canopen"]
MULTI_CONF = True

CONF_HEARTBEAT_TIMEOUT = "heartbeat_timeout"

ns = cg.esphome_ns.namespace("canopen_test_profile")
TestProfile = ns.class_("TestProfile", cg.Component, canopen.CANopenNode)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TestProfile),
            cv.Optional(CONF_HEARTBEAT_TIMEOUT): canopen.UINT16_MS,
        }
    )
    .extend(canopen.CANOPEN_NODE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)
FINAL_VALIDATE_SCHEMA = canopen.final_validate_canopen_node


async def to_code(config):
    hub = await cg.get_variable(config[canopen.CONF_CANOPEN_ID])
    var = cg.new_Pvariable(config[CONF_ID], hub, config[canopen.CONF_NODE_ID])
    await cg.register_component(var, config)
    # test glue: own PDOs for can2mqtt_test.h (TPDO of the fake sensor, RPDO of node 0x63's TPDO)
    canopen.enable_pdo()
    await canopen.register_node(
        var, config, sdo=True, nmt=True, heartbeat_timeout=config.get(CONF_HEARTBEAT_TIMEOUT)
    )

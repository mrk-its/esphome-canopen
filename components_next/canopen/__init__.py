from pathlib import Path

import esphome.config_validation as cv
import esphome.codegen as cg
from esphome import automation
from esphome.const import CONF_ID, CONF_TRIGGER_ID
from esphome.components.canbus import CanbusComponent

DEPENDENCIES = ["canbus"]
MULTI_CONF = True

CONF_CANBUS_ID = "canbus_id"
CONF_NODE_ID = "node_id"
CONF_HEARTBEAT_INTERVAL = "heartbeat_interval"
CONF_HEARTBEAT_CLIENTS = "heartbeat_clients"
CONF_TIMEOUT = "timeout"
CONF_ON_HB_CONSUMER_EVENT = "on_hb_consumer_event"
CONF_SDO_BLOCK_TRANSFER_SIZE = "sdo_block_transfer_size"
CONF_CANOPENNODE_PATH = "canopennode_path"

# CANopenNode fork checked out next to this repo
DEFAULT_CANOPENNODE_PATH = Path(__file__).resolve().parents[3] / "CANopenNode"

ns = cg.esphome_ns.namespace("canopen")
CanopenComponent = ns.class_("CanopenComponent", cg.Component)
HbConsumerEventTrigger = ns.class_(
    "HbConsumerEventTrigger", automation.Trigger.template(cg.uint8)
)


def absolute_path(value):
    # the zephyr library converter rejects relative file:// sources
    return str(Path(cv.string(value)).expanduser().resolve())


HB_CLIENT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_NODE_ID): cv.All(cv.int_, cv.Range(min=1, max=127)),
        cv.Required(CONF_TIMEOUT): cv.positive_time_period_milliseconds,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CanopenComponent),
        cv.GenerateID(CONF_CANBUS_ID): cv.use_id(CanbusComponent),
        cv.Required(CONF_NODE_ID): cv.All(cv.int_, cv.Range(min=1, max=127)),
        cv.Optional(
            CONF_HEARTBEAT_INTERVAL, "5000ms"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_HEARTBEAT_CLIENTS, []): cv.ensure_list(HB_CLIENT_SCHEMA),
        cv.Optional(CONF_ON_HB_CONSUMER_EVENT): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(HbConsumerEventTrigger)}
        ),
        cv.Optional(CONF_SDO_BLOCK_TRANSFER_SIZE, 63): cv.All(
            cv.int_, cv.Range(min=1, max=127)
        ),
        cv.Optional(
            CONF_CANOPENNODE_PATH, str(DEFAULT_CANOPENNODE_PATH)
        ): absolute_path,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    cg.add_library("CANopenNode", None, f"file://{config[CONF_CANOPENNODE_PATH]}")
    cg.add_build_flag(
        f"-DCO_CONFIG_SDO_SRV_BLKSIZE={config[CONF_SDO_BLOCK_TRANSFER_SIZE]}"
    )

    var = cg.new_Pvariable(config[CONF_ID], config[CONF_NODE_ID])
    await cg.register_component(var, config)

    canbus = await cg.get_variable(config[CONF_CANBUS_ID])
    cg.add(var.set_canbus(canbus))
    cg.add(var.set_heartbeat_interval(config[CONF_HEARTBEAT_INTERVAL]))
    for client in config[CONF_HEARTBEAT_CLIENTS]:
        cg.add(
            var.add_heartbeat_consumer(
                client[CONF_NODE_ID], client[CONF_TIMEOUT].total_milliseconds
            )
        )

    for conf in config.get(CONF_ON_HB_CONSUMER_EVENT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        await automation.build_automation(trigger, [(cg.uint8, "node_id")], conf)
        cg.add(var.set_hb_consumer_event_trigger(trigger))

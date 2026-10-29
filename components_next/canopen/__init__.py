from pathlib import Path

import esphome.config_validation as cv
import esphome.codegen as cg
from esphome import automation
from esphome.const import CONF_ID, CONF_TRIGGER_ID
from esphome.core import CORE
from esphome.components.canbus import CanbusComponent

DEPENDENCIES = ["canbus"]
MULTI_CONF = True

CONF_CANBUS_ID = "canbus_id"
# for components that refer to a canopen node (cv.use_id(CANopenComponent))
CONF_CANOPEN_ID = "canopen_id"
CONF_NODE_ID = "node_id"
CONF_HEARTBEAT_INTERVAL = "heartbeat_interval"
CONF_HEARTBEAT_CLIENTS = "heartbeat_clients"
CONF_TIMEOUT = "timeout"
CONF_ON_HB_CONSUMER_EVENT = "on_hb_consumer_event"
CONF_ON_HEARTBEAT = "on_heartbeat"
CONF_ON_EMCY = "on_emcy"
CONF_CANOPENNODE_PATH = "canopennode_path"

# CANopenNode upstream + library.json (branch esphome), pinned to a commit
CANOPENNODE_SOURCE = (
    "https://github.com/mrk-its/CANopenNode.git#f38ec8a6442a5a598d26f88c4e7c8a39b83d2cd5"
)
# directory name = IDF component name (a local IDF component is named after its directory)
PORT_PATH = Path(__file__).resolve().parent / "canopennode_port"

ns = cg.esphome_ns.namespace("canopen")
CANopenComponent = ns.class_("CANopenComponent", cg.Component)
HbConsumerEventTrigger = ns.class_(
    "HbConsumerEventTrigger", automation.Trigger.template(cg.uint8)
)
NMTState = ns.enum("NMTState", is_class=True)
Emergency = ns.struct("Emergency")

CALLBACK_AUTOMATIONS = (
    automation.CallbackAutomation(
        CONF_ON_HEARTBEAT,
        "add_on_heartbeat_callback",
        [(cg.uint8, "node_id"), (NMTState, "state")],
    ),
    automation.CallbackAutomation(
        CONF_ON_EMCY,
        "add_on_emcy_callback",
        [(cg.uint8, "node_id"), (Emergency.operator("const").operator("ref"), "emergency")],
    ),
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
        cv.GenerateID(): cv.declare_id(CANopenComponent),
        cv.GenerateID(CONF_CANBUS_ID): cv.use_id(CanbusComponent),
        cv.Required(CONF_NODE_ID): cv.All(cv.int_, cv.Range(min=1, max=127)),
        cv.Optional(
            CONF_HEARTBEAT_INTERVAL, "5000ms"
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_HEARTBEAT_CLIENTS, []): cv.ensure_list(HB_CLIENT_SCHEMA),
        cv.Optional(CONF_ON_HB_CONSUMER_EVENT): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(HbConsumerEventTrigger)}
        ),
        # every heartbeat / EMCY frame on the bus, any node
        cv.Optional(CONF_ON_HEARTBEAT): automation.validate_automation({}),
        cv.Optional(CONF_ON_EMCY): automation.validate_automation({}),
        # local CANopenNode checkout instead of CANOPENNODE_SOURCE (development)
        cv.Optional(CONF_CANOPENNODE_PATH): absolute_path,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    if CONF_CANOPENNODE_PATH in config:
        cg.add_library("CANopenNode", None, f"file://{config[CONF_CANOPENNODE_PATH]}")
    else:
        cg.add_library("CANopenNode", None, CANOPENNODE_SOURCE)
    # CANopenNode's 301/CO_driver.h includes CO_driver_target.h, which lives in canopennode_port/. Project -I
    # flags don't reach converted libraries (only -D/-W do), so that directory is added as a package of its own:
    if CORE.is_esp32:
        # ESP-IDF: a converted library sees only the components it requires, which include every
        # project-managed IDF component
        from esphome.components.esp32 import add_idf_component

        add_idf_component(name="canopennode_port", path=str(PORT_PATH))
    else:
        # Zephyr (native build) exports every library's include dirs globally
        cg.add_library("CANopenNode-esphome-port", None, f"file://{PORT_PATH}")
        # PlatformIO (ESP8266, ...) passes project build flags to libraries, but its LDF doesn't link the
        # library above to CANopenNode
        cg.add_build_flag(f"-I{PORT_PATH}")

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

    await automation.build_callback_automations(var, config, CALLBACK_AUTOMATIONS)

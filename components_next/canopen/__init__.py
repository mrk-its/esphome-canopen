from pathlib import Path

from esphome import automation
import esphome.codegen as cg
from esphome.components.canbus import CanbusComponent
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TIMEOUT
from esphome.core import CORE

CODEOWNERS = ["@mrk-its"]
DEPENDENCIES = ["canbus"]
DOMAIN = "canopen"
MULTI_CONF = True

CONF_CANBUS_ID = "canbus_id"
# for components that refer to a canopen node (cv.use_id(CANopenComponent))
CONF_CANOPEN_ID = "canopen_id"
CONF_NODE_ID = "node_id"
CONF_HEARTBEAT_INTERVAL = "heartbeat_interval"
CONF_HEARTBEAT_CLIENTS = "heartbeat_clients"
CONF_ON_NODE_EVENT = "on_node_event"
CONF_ON_HEARTBEAT = "on_heartbeat"
CONF_ON_EMCY = "on_emcy"
CONF_CANOPENNODE_PATH = "canopennode_path"

# CANopenNode upstream + library.json (branch esphome), pinned to a commit
CANOPENNODE_SOURCE = "https://github.com/mrk-its/CANopenNode.git#f38ec8a6442a5a598d26f88c4e7c8a39b83d2cd5"
# directory name = IDF component name (a local IDF component is named after its directory)
PORT_PATH = Path(__file__).resolve().parent / "canopennode_port"

ns = cg.esphome_ns.namespace("canopen")
CANopenComponent = ns.class_("CANopenComponent", cg.Component)
NMTState = ns.enum("NMTState", is_class=True)
NodeEvent = ns.enum("NodeEvent", is_class=True)
Emergency = ns.struct("Emergency")

CALLBACK_AUTOMATIONS = (
    automation.CallbackAutomation(
        CONF_ON_NODE_EVENT,
        "add_on_node_event_callback",
        [(cg.uint8, "node_id"), (NodeEvent, "event"), (NMTState, "state")],
    ),
    automation.CallbackAutomation(
        CONF_ON_HEARTBEAT,
        "add_on_heartbeat_callback",
        [(cg.uint8, "node_id"), (NMTState, "state")],
    ),
    automation.CallbackAutomation(
        CONF_ON_EMCY,
        "add_on_emcy_callback",
        [
            (cg.uint8, "node_id"),
            (Emergency.operator("const").operator("ref"), "emergency"),
        ],
    ),
)


def absolute_path(value):
    # the zephyr library converter rejects relative file:// sources
    return str(Path(cv.string(value)).expanduser().resolve())


HB_CLIENT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_NODE_ID): cv.All(cv.int_, cv.Range(min=1, max=127)),
        # 0x1016 holds the timeout as uint16 ms
        cv.Required(CONF_TIMEOUT): cv.All(
            cv.positive_time_period_milliseconds,
            cv.Range(
                min=cv.TimePeriod(milliseconds=1), max=cv.TimePeriod(milliseconds=65535)
            ),
        ),
    }
)


def validate_heartbeat_clients(config):
    for client in config[CONF_HEARTBEAT_CLIENTS]:
        if client[CONF_NODE_ID] == config[CONF_NODE_ID]:
            raise cv.Invalid(
                f"heartbeat_clients: node_id {client[CONF_NODE_ID]} is this node's own id"
            )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CANopenComponent),
            cv.GenerateID(CONF_CANBUS_ID): cv.use_id(CanbusComponent),
            cv.Required(CONF_NODE_ID): cv.All(cv.int_, cv.Range(min=1, max=127)),
            cv.Optional(
                CONF_HEARTBEAT_INTERVAL, "5000ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_HEARTBEAT_CLIENTS, []): cv.ensure_list(HB_CLIENT_SCHEMA),
            # events of the heartbeat_clients nodes (online, offline, NMT changed, reset)
            cv.Optional(CONF_ON_NODE_EVENT): automation.validate_automation({}),
            # every heartbeat / EMCY frame on the bus, any node
            cv.Optional(CONF_ON_HEARTBEAT): automation.validate_automation({}),
            cv.Optional(CONF_ON_EMCY): automation.validate_automation({}),
            # local CANopenNode checkout instead of CANOPENNODE_SOURCE (development)
            cv.Optional(CONF_CANOPENNODE_PATH): absolute_path,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_heartbeat_clients,
)


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
            var.monitor_node(
                client[CONF_NODE_ID], client[CONF_TIMEOUT].total_milliseconds
            )
        )

    await automation.build_callback_automations(var, config, CALLBACK_AUTOMATIONS)

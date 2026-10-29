from pathlib import Path

from esphome import automation
import esphome.codegen as cg
from esphome.components.canbus import CanbusComponent
import esphome.config_validation as cv
from esphome.const import (
    CONF_COMMAND,
    CONF_ID,
    CONF_INDEX,
    CONF_ON_ERROR,
    CONF_ON_SUCCESS,
    CONF_SIZE,
    CONF_TIMEOUT,
    CONF_VALUE,
)
from esphome.core import CORE
from esphome.helpers import write_file_if_changed

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
CONF_RESPONSE_TIMEOUT = "response_timeout"
CONF_SUBINDEX = "subindex"

# CANopenNode upstream + library.json (branch esphome), pinned to a commit
CANOPENNODE_SOURCE = "https://github.com/mrk-its/CANopenNode.git#f38ec8a6442a5a598d26f88c4e7c8a39b83d2cd5"
PORT_PATH = Path(__file__).resolve().parent / "canopennode_port"
PORT_HEADER = "CO_driver_target.h"
# a local IDF component is named after its directory
IDF_PORT_COMPONENT = "canopennode_port"

ns = cg.esphome_ns.namespace("canopen")
CANopenComponent = ns.class_("CANopenComponent", cg.Component)
NMTState = ns.enum("NMTState", is_class=True)
NodeEvent = ns.enum("NodeEvent", is_class=True)
Emergency = ns.struct("Emergency")
SDOResult = ns.enum("SDOResult", is_class=True)
NMTCommand = ns.enum("NMTCommand", is_class=True)
SDOWriteAction = ns.class_("SDOWriteAction", automation.Action)
NMTAction = ns.class_("NMTAction", automation.Action)

NMT_COMMANDS = {
    "start": NMTCommand.START,
    "stop": NMTCommand.STOP,
    "pre_operational": NMTCommand.PRE_OPERATIONAL,
    "reset_node": NMTCommand.RESET_NODE,
    "reset_communication": NMTCommand.RESET_COMMUNICATION,
}

KEY_CANOPEN = "canopen"


def _enable_once(key: str) -> bool:
    enabled = CORE.data.setdefault(KEY_CANOPEN, set())
    if key in enabled:
        return False
    enabled.add(key)
    return True


def enable_sdo_client() -> None:
    """Compile the SDO client (read_sdo(), write_sdo(), ...): for actions and components that use it."""
    if not _enable_once("sdo_client"):
        return
    cg.add_define("USE_CANOPEN_SDO_CLIENT")
    # enable | segmented | block | local (requests to our own node id use our OD); the client needs the fifo
    # (enable | alt read | crc16 ccitt) and crc16
    cg.add_build_flag("-DCO_CONFIG_SDO_CLI=0x0F")
    cg.add_build_flag("-DCO_CONFIG_FIFO=0x07")
    cg.add_build_flag("-DCO_CONFIG_CRC16=0x01")


def enable_nmt_master() -> None:
    """Compile the NMT master (send_nmt()): for actions and components that use it."""
    if not _enable_once("nmt_master"):
        return
    cg.add_define("USE_CANOPEN_NMT_MASTER")
    # callback change | master
    cg.add_build_flag("-DCO_CONFIG_NMT=0x03")


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


# 0x1016 (consumer timeout) and 0x1017 (producer time) hold uint16 ms
UINT16_MS = cv.All(
    cv.positive_time_period_milliseconds,
    cv.Range(min=cv.TimePeriod(milliseconds=1), max=cv.TimePeriod(milliseconds=65535)),
)

HB_CLIENT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_NODE_ID): cv.All(cv.int_, cv.Range(min=1, max=127)),
        cv.Required(CONF_TIMEOUT): UINT16_MS,
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
            cv.Optional(CONF_HEARTBEAT_INTERVAL, "5000ms"): UINT16_MS,
            cv.Optional(CONF_HEARTBEAT_CLIENTS, []): cv.ensure_list(HB_CLIENT_SCHEMA),
            # events of the heartbeat_clients nodes (online, offline, NMT changed, reset)
            cv.Optional(CONF_ON_NODE_EVENT): automation.validate_automation({}),
            # every heartbeat / EMCY frame on the bus, any node
            cv.Optional(CONF_ON_HEARTBEAT): automation.validate_automation({}),
            cv.Optional(CONF_ON_EMCY): automation.validate_automation({}),
            # local CANopenNode checkout instead of CANOPENNODE_SOURCE (development)
            cv.Optional(CONF_RESPONSE_TIMEOUT, "500ms"): UINT16_MS,
            cv.Optional(CONF_CANOPENNODE_PATH): absolute_path,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_heartbeat_clients,
)


def add_canopennode_library(path: str | None = None) -> None:
    """Add the CANopenNode library and its target header (also used by the C++ unit tests)."""
    if path is not None:
        cg.add_library("CANopenNode", None, f"file://{path}")
    else:
        cg.add_library("CANopenNode", None, CANOPENNODE_SOURCE)
    # CANopenNode's 301/CO_driver.h includes CO_driver_target.h, which lives in canopennode_port/. Project -I
    # flags don't reach converted libraries (only -D/-W do), so that directory is added as a package of its own:
    if CORE.is_esp32:
        # ESP-IDF: a converted library sees only the components it requires, which include every
        # project-managed IDF component. Generated in the build dir: CMakeLists.txt files are gitignored in esphome.
        from esphome.components.esp32 import add_idf_component

        idf_port = CORE.relative_build_path(IDF_PORT_COMPONENT)
        idf_port.mkdir(parents=True, exist_ok=True)
        write_file_if_changed(
            idf_port / PORT_HEADER,
            (PORT_PATH / PORT_HEADER).read_text(encoding="utf-8"),
        )
        write_file_if_changed(
            idf_port / "CMakeLists.txt", 'idf_component_register(INCLUDE_DIRS ".")\n'
        )
        add_idf_component(name=IDF_PORT_COMPONENT, path=str(idf_port))
    else:
        # Zephyr (native build) exports every library's include dirs globally
        cg.add_library("CANopenNode-esphome-port", None, f"file://{PORT_PATH}")
        # PlatformIO (ESP8266, host, ...) passes project build flags to libraries, but its LDF doesn't link the
        # library above to CANopenNode
        cg.add_build_flag(f"-I{PORT_PATH}")


async def to_code(config):
    add_canopennode_library(config.get(CONF_CANOPENNODE_PATH))

    var = cg.new_Pvariable(config[CONF_ID], config[CONF_NODE_ID])
    await cg.register_component(var, config)

    canbus = await cg.get_variable(config[CONF_CANBUS_ID])
    cg.add(var.set_canbus(canbus))
    cg.add(var.set_heartbeat_interval(config[CONF_HEARTBEAT_INTERVAL]))
    cg.add(var.set_response_timeout(config[CONF_RESPONSE_TIMEOUT]))
    for client in config[CONF_HEARTBEAT_CLIENTS]:
        cg.add(
            var.monitor_node(
                client[CONF_NODE_ID], client[CONF_TIMEOUT].total_milliseconds
            )
        )

    await automation.build_callback_automations(var, config, CALLBACK_AUTOMATIONS)


def validate_sdo_value(config):
    value = config[CONF_VALUE]
    if isinstance(value, int):
        bits = 8 * config[CONF_SIZE]
        if not -(1 << (bits - 1)) <= value < (1 << bits):
            raise cv.Invalid(
                f"value {value} doesn't fit in {config[CONF_SIZE]} byte(s)"
            )
    return config


SDO_WRITE_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(CANopenComponent),
            cv.Required(CONF_NODE_ID): cv.templatable(cv.int_range(min=1, max=127)),
            cv.Required(CONF_INDEX): cv.templatable(cv.hex_uint16_t),
            cv.Optional(CONF_SUBINDEX, default=0): cv.templatable(cv.hex_uint8_t),
            cv.Required(CONF_VALUE): cv.templatable(
                cv.int_range(min=-(1 << 31), max=(1 << 32) - 1)
            ),
            cv.Optional(CONF_SIZE, default=4): cv.one_of(1, 2, 4, int=True),
            cv.Optional(CONF_ON_SUCCESS): automation.validate_automation({}),
            cv.Optional(CONF_ON_ERROR): automation.validate_automation({}),
        }
    ),
    validate_sdo_value,
)


@automation.register_action(
    "canopen.sdo_write", SDOWriteAction, SDO_WRITE_SCHEMA, synchronous=True
)
async def sdo_write_to_code(config, action_id, template_arg, args):
    enable_sdo_client()
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    for key, setter, type_ in (
        (CONF_NODE_ID, var.set_node_id, cg.uint8),
        (CONF_INDEX, var.set_index, cg.uint16),
        (CONF_SUBINDEX, var.set_subindex, cg.uint8),
    ):
        cg.add(setter(await cg.templatable(config[key], args, type_)))
    value = config[CONF_VALUE]
    if isinstance(value, int):
        value &= 0xFFFFFFFF
    cg.add(var.set_value(await cg.templatable(value, args, cg.uint32)))
    cg.add(var.set_size(config[CONF_SIZE]))
    for conf in config.get(CONF_ON_SUCCESS, []):
        await automation.build_automation(var.get_success_trigger(), [], conf)
    for conf in config.get(CONF_ON_ERROR, []):
        await automation.build_automation(
            var.get_error_trigger(),
            [(SDOResult, "result"), (cg.uint32, "abort_code")],
            conf,
        )
    return var


NMT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(CANopenComponent),
        # 0 = all nodes
        cv.Required(CONF_NODE_ID): cv.templatable(cv.int_range(min=0, max=127)),
        cv.Required(CONF_COMMAND): cv.enum(NMT_COMMANDS, lower=True),
    }
)


@automation.register_action("canopen.nmt", NMTAction, NMT_SCHEMA, synchronous=True)
async def nmt_to_code(config, action_id, template_arg, args):
    enable_nmt_master()
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_node_id(await cg.templatable(config[CONF_NODE_ID], args, cg.uint8)))
    cg.add(var.set_command(config[CONF_COMMAND]))
    return var

from pathlib import Path

from esphome import automation
import esphome.codegen as cg
from esphome.components.canbus import CanbusComponent
from esphome.components.const import CONF_SERIAL_NUMBER
import esphome.config_validation as cv
from esphome.const import (
    CONF_COMMAND,
    CONF_ID,
    CONF_IDENTITY,
    CONF_INDEX,
    CONF_ON_ERROR,
    CONF_ON_SUCCESS,
    CONF_SIZE,
    CONF_TIMEOUT,
    CONF_VALUE,
)
from esphome.core import CORE, ID
from esphome.cpp_generator import MockObj, TemplateArgsType
import esphome.final_validate as fv
from esphome.helpers import write_file_if_changed
from esphome.types import ConfigType

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
CONF_ON_NMT_STATE_CHANGE = "on_nmt_state_change"
CONF_NMT_STARTUP = "nmt_startup"
CONF_SDO_BLOCK_SIZE = "sdo_block_size"
CONF_CANOPENNODE_PATH = "canopennode_path"
CONF_RESPONSE_TIMEOUT = "response_timeout"
CONF_SUBINDEX = "subindex"
CONF_VENDOR_ID = "vendor_id"
CONF_PRODUCT_CODE = "product_code"
CONF_REVISION_NUMBER = "revision_number"
CONF_DEVICE_NAME = "device_name"
CONF_HARDWARE_VERSION = "hardware_version"
CONF_SOFTWARE_VERSION = "software_version"

# CANopenNode upstream + library.json + SDO server block size option (branch esphome), pinned to a commit
CANOPENNODE_SOURCE = "https://github.com/mrk-its/CANopenNode.git#9ff367b8205d746a84111b38e01ace3feaffb401"
PORT_PATH = Path(__file__).resolve().parent / "canopennode_port"
PORT_HEADER = "CO_driver_target.h"
# a local IDF component is named after its directory
IDF_PORT_COMPONENT = "canopennode_port"

ns = cg.esphome_ns.namespace("canopen")
CANopenComponent = ns.class_("CANopenComponent", cg.Component)
# base class of components that drive another node (canopen_node.h)
CANopenNode = ns.class_("CANopenNode")
NMTState = ns.enum("NMTState", is_class=True)
NodeEvent = ns.enum("NodeEvent", is_class=True)
Emergency = ns.struct("Emergency")
SDOResult = ns.enum("SDOResult", is_class=True)
NMTCommand = ns.enum("NMTCommand", is_class=True)
SDOWriteAction = ns.class_("SDOWriteAction", automation.Action)

NMT_COMMANDS = {
    "start": NMTCommand.NMT_COMMAND_START,
    "stop": NMTCommand.NMT_COMMAND_STOP,
    "pre_operational": NMTCommand.NMT_COMMAND_PRE_OPERATIONAL,
    "reset_node": NMTCommand.NMT_COMMAND_RESET_NODE,
    "reset_communication": NMTCommand.NMT_COMMAND_RESET_COMMUNICATION,
}

KEY_CANOPEN = "canopen"
# canbus id -> the canopen nodes on it, in config order (to_code links them, see set_next_local())
KEY_LOCAL_NODES = "canopen_local_nodes"


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
    # enable | segmented | local (requests to our own node id use our OD), with the fifo. No block transfer: it's
    # optional in CiA 301, a server without it aborts the request, and the client doesn't fall back to segmented.
    cg.add_build_flag("-DCO_CONFIG_SDO_CLI=0x0B")
    cg.add_build_flag("-DCO_CONFIG_FIFO=0x01")


# CANopenNode's minimum server buffer for block transfer (CO_SDOserver.c checks it)
SDO_BLOCK_BUFFER_SIZE = 900


def enable_sdo_block(block_size: int) -> None:
    """SDO server block transfer (CiA 301), e.g. for firmware or log uploads.

    `block_size` (1..127) caps the segments per block the server announces for downloads (the client sends a
    block back to back, so it must fit the CAN receive path); for uploads the client chooses it. Without block
    transfer the server is segmented only, with a 32-byte buffer.

    These are CANopenNode build flags, so they apply to every canopen node of the firmware: a node's
    sdo_block_size (validated to be the same on all of them) wins over `block_size`, otherwise the first call
    sets it.
    """
    if not _enable_once("sdo_block"):
        return
    configured = {
        hub[CONF_SDO_BLOCK_SIZE]
        for hub in CORE.config.get(DOMAIN, [])
        if CONF_SDO_BLOCK_SIZE in hub
    }
    if configured:
        block_size = configured.pop()
    cg.add_define("CANOPEN_SDO_BLOCK_SIZE", block_size)
    # segmented | block, the buffer for a whole block, the download block size cap (CO_CONFIG_SDO_SRV_BLKSIZE needs
    # the SDO server block size patch of the pinned CANopenNode), crc16 for the block CRC
    cg.add_build_flag("-DCO_CONFIG_SDO_SRV=0x06")
    cg.add_build_flag(f"-DCO_CONFIG_SDO_SRV_BUFFER_SIZE={SDO_BLOCK_BUFFER_SIZE}")
    cg.add_build_flag(f"-DCO_CONFIG_SDO_SRV_BLKSIZE={block_size}")
    cg.add_build_flag("-DCO_CONFIG_CRC16=0x01")


def enable_pdo() -> None:
    """Compile our own PDOs (configure_tpdo(), configure_rpdo(), request_tpdo()): for components that use them."""
    if not _enable_once("pdo"):
        return
    cg.add_define("USE_CANOPEN_PDO")
    # RPDO | TPDO | RPDO timers | TPDO timers | OD IO access (RPDO data reaches the handlers) | OD dynamic (records
    # changed over SDO); no SYNC
    cg.add_build_flag("-DCO_CONFIG_PDO=0x402F")
    # 0x1010 / 0x1011 store / restore of the PDO parameters (CO_storage, our callbacks on preferences)
    cg.add_build_flag("-DCO_CONFIG_STORAGE=0x01")


def enable_nmt_master() -> None:
    """Compile the NMT master (send_nmt()): for actions and components that use it."""
    if not _enable_once("nmt_master"):
        return
    cg.add_define("USE_CANOPEN_NMT_MASTER")
    # callback change | master
    cg.add_build_flag("-DCO_CONFIG_NMT=0x03")


# for components that drive another node (subclasses of CANopenNode), see register_node()
CANOPEN_NODE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CANOPEN_ID): cv.use_id(CANopenComponent),
        cv.Required(CONF_NODE_ID): cv.All(cv.int_, cv.Range(min=1, max=127)),
    }
)


def final_validate_canopen_node(config):
    """FINAL_VALIDATE_SCHEMA helper for CANOPEN_NODE_SCHEMA: the node can't be the hub's own node id."""
    full_config = fv.full_config.get()
    path = full_config.get_path_for_id(config[CONF_CANOPEN_ID])[:-1]
    hub = full_config.get_config_for_path(path)
    if hub[CONF_NODE_ID] == config[CONF_NODE_ID]:
        raise cv.Invalid(
            f"node_id {config[CONF_NODE_ID]} is the own id of canopen {config[CONF_CANOPEN_ID]}",
            path=[CONF_NODE_ID],
        )
    return config


async def register_node(
    var, config, sdo: bool = False, nmt: bool = False, heartbeat_timeout=None
):
    """Codegen for a CANopenNode subclass built from CANOPEN_NODE_SCHEMA.

    Compiles the parts of the hub the component uses (SDO client, NMT master) and, with a heartbeat_timeout
    (TimePeriod, validated with UINT16_MS), monitors the node: its events reach the on_node_* hooks. The hub merges it with
    heartbeat_clients, keeping the shortest timeout.
    """
    if sdo:
        enable_sdo_client()
    if nmt:
        enable_nmt_master()
    if heartbeat_timeout is not None:
        hub = await cg.get_variable(config[CONF_CANOPEN_ID])
        cg.add(
            hub.monitor_node(config[CONF_NODE_ID], heartbeat_timeout.total_milliseconds)
        )
    return var


CALLBACK_AUTOMATIONS = (
    automation.CallbackAutomation(
        CONF_ON_NODE_EVENT,
        "add_on_node_event_callback",
        [(cg.uint8, "node_id"), (NodeEvent, "event"), (NMTState, "state")],
    ),
    automation.CallbackAutomation(
        CONF_ON_NMT_STATE_CHANGE,
        "add_on_nmt_state_callback",
        [(NMTState, "state")],
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

# 0x1018; vendor ids are assigned by CiA
IDENTITY_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_VENDOR_ID, default=0): cv.hex_uint32_t,
        cv.Optional(CONF_PRODUCT_CODE, default=0): cv.hex_uint32_t,
        cv.Optional(CONF_REVISION_NUMBER, default=0): cv.hex_uint32_t,
        cv.Optional(CONF_SERIAL_NUMBER, default=0): cv.hex_uint32_t,
    }
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
            cv.Optional(CONF_RESPONSE_TIMEOUT, "500ms"): UINT16_MS,
            # SDO server block transfer, see enable_sdo_block(); without it segmented only. Firmware-wide: the
            # same on every canopen node
            cv.Optional(CONF_SDO_BLOCK_SIZE): cv.int_range(min=1, max=127),
            # pre_operational: until an NMT master (or set_nmt_state()) starts the node
            cv.Optional(CONF_NMT_STARTUP, default="operational"): cv.one_of(
                "operational", "pre_operational", lower=True
            ),
            # our own NMT state
            cv.Optional(CONF_ON_NMT_STATE_CHANGE): automation.validate_automation({}),
            cv.Optional(CONF_IDENTITY, default={}): IDENTITY_SCHEMA,
            # 0x1008 (default: the node's name), 0x1009, 0x100A (default: the ESPHome version)
            cv.Optional(CONF_DEVICE_NAME): cv.string,
            cv.Optional(CONF_HARDWARE_VERSION): cv.string,
            cv.Optional(CONF_SOFTWARE_VERSION): cv.string,
            # local CANopenNode checkout instead of CANOPENNODE_SOURCE (development)
            cv.Optional(CONF_CANOPENNODE_PATH): absolute_path,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_heartbeat_clients,
)


def _final_validate(config):
    for other in fv.full_config.get().get(DOMAIN, []):
        # CANopenNode build flags (enable_sdo_block()): one value for the whole firmware
        if (
            CONF_SDO_BLOCK_SIZE in config
            and CONF_SDO_BLOCK_SIZE in other
            and other[CONF_SDO_BLOCK_SIZE] != config[CONF_SDO_BLOCK_SIZE]
        ):
            raise cv.Invalid(
                f"sdo_block_size applies to every canopen node of the firmware: {config[CONF_SDO_BLOCK_SIZE]} "
                f"here, {other[CONF_SDO_BLOCK_SIZE]} on canopen {other[CONF_ID]}",
                path=[CONF_SDO_BLOCK_SIZE],
            )
        # nodes on the same canbus receive each other's frames: their ids must differ, as on any CANopen bus
        if (
            other[CONF_ID] != config[CONF_ID]
            and other[CONF_CANBUS_ID] == config[CONF_CANBUS_ID]
            and other[CONF_NODE_ID] == config[CONF_NODE_ID]
        ):
            raise cv.Invalid(
                f"node_id {config[CONF_NODE_ID]} is also used by canopen {other[CONF_ID]} on the same canbus",
                path=[CONF_NODE_ID],
            )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


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
    # other nodes on this canbus: a ring in config order (each call closes it again)
    local = CORE.data.setdefault(KEY_LOCAL_NODES, {}).setdefault(
        str(config[CONF_CANBUS_ID]), []
    )
    if local:
        cg.add(local[-1].set_next_local(var))
        cg.add(var.set_next_local(local[0]))
    local.append(var)
    cg.add(var.set_heartbeat_interval(config[CONF_HEARTBEAT_INTERVAL]))
    cg.add(var.set_response_timeout(config[CONF_RESPONSE_TIMEOUT]))
    if config[CONF_NMT_STARTUP] != "operational":
        cg.add(var.set_start_operational(False))
    if CONF_SDO_BLOCK_SIZE in config:
        enable_sdo_block(config[CONF_SDO_BLOCK_SIZE])
    identity = config[CONF_IDENTITY]
    cg.add(
        var.set_identity(
            identity[CONF_VENDOR_ID],
            identity[CONF_PRODUCT_CODE],
            identity[CONF_REVISION_NUMBER],
            identity[CONF_SERIAL_NUMBER],
        )
    )
    for key, setter in (
        (CONF_DEVICE_NAME, var.set_device_name),
        (CONF_HARDWARE_VERSION, var.set_hardware_version),
        (CONF_SOFTWARE_VERSION, var.set_software_version),
    ):
        if key in config:
            cg.add(setter(config[key]))
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
        # negative values as two's complement in `size` bytes
        value &= (1 << (8 * config[CONF_SIZE])) - 1
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


_nmt_builder = automation.apply_action_builder(
    "canopen.nmt",
    NMT_SCHEMA,
    automation.ApplyCall(
        "send_nmt({}, {})", ((CONF_NODE_ID, cg.uint8), (CONF_COMMAND, NMTCommand))
    ),
)


@automation.register_action(
    "canopen.nmt", automation.ApplyAction, NMT_SCHEMA, synchronous=True
)
async def nmt_to_code(
    config: ConfigType,
    action_id: ID,
    template_arg: cg.TemplateArguments,
    args: TemplateArgsType,
) -> MockObj:
    enable_nmt_master()
    return await _nmt_builder(config, action_id, template_arg, args)

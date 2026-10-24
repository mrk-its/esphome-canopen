import datetime
from itertools import groupby
import logging

import esphome.config_validation as cv
import esphome.codegen as cg
from esphome import automation
import esphome.final_validate as fv
from esphome.const import CONF_ID, CONF_TRIGGER_ID
from esphome.components.canbus import CanbusComponent
from esphome.core import CORE, Lambda

from . import layout, remote

_LOGGER = logging.getLogger(__name__)

ns = cg.esphome_ns.namespace("canopen")

TPDO = ns.struct("TPDO")

CanopenComponent = ns.class_(
    "CanopenComponent",
    cg.Component,
)

PreOperationalTrigger = ns.class_(
    "PreOperationalTrigger", automation.Trigger.template()
)
OperationalTrigger = ns.class_("OperationalTrigger", automation.Trigger.template())
HbConsumerEventTrigger = ns.class_(
    "HbConsumerEventTrigger", automation.Trigger.template()
)
CmdTriggerUInt8 = ns.class_("CmdTriggerUInt8", automation.Trigger.template())
CmdTriggerUInt16 = ns.class_("CmdTriggerUInt16", automation.Trigger.template())
CmdTriggerUInt32 = ns.class_("CmdTriggerUInt32", automation.Trigger.template())
CmdTriggerInt8 = ns.class_("CmdTriggerInt8", automation.Trigger.template())
CmdTriggerInt16 = ns.class_("CmdTriggerInt16", automation.Trigger.template())
CmdTriggerInt32 = ns.class_("CmdTriggerInt32", automation.Trigger.template())
EntityCmdAction = ns.class_("EntityCmdAction", automation.Action)
RemoteCommandAction = ns.class_("RemoteCommandAction", automation.Action)

CONF_ENTITIES = "entities"
CONF_REMOTE_NODES = "remote_nodes"
CONF_RPDO = "rpdo"
CONF_CANOPEN_ID = "canopen_id"

# remote state (node, entity, state) a `platform: canopen` entity or a toggle
# action reads, set by their validation; collected by remote_state_slots()
CONF_REMOTE_STATE = "_canopen_remote_state"
REMOTE_STATE_INDEX = 0x4000  # OD index of slot 0, see OBJECT_DICTIONARY.md

CONF_REMOTE_LOGS = "remote_logs"
# logger's level names, the index is the ESPHOME_LOG_LEVEL_* value (OD 0x3100:3 / :5)
LOG_LEVEL_SEVERITY = [
    "NONE",
    "ERROR",
    "WARN",
    "INFO",
    "CONFIG",
    "DEBUG",
    "VERBOSE",
    "VERY_VERBOSE",
]
# SDO block size for log uploads without a software TX queue: the 3 TX mailboxes of bxCAN / FDCAN
DEFAULT_LOG_BLOCK_SIZE = 3

DEPENDENCIES = []

CSDO_SCHEMA = cv.Schema(
    {
        cv.Required("node_id"): cv.int_,
        cv.Optional("tx_id", 0x600): cv.int_,
        cv.Optional("rx_id", 0x580): cv.int_,
    }
)

RPDO_RAW_SCHEMA = cv.Schema(
    {
        cv.Required("node_id"): cv.int_,
        cv.Required("tpdo"): cv.int_,
        cv.Required("offset"): cv.int_,
        cv.Optional("cmd", 0): cv.int_,
    }
)

# remote state by node name (remote_nodes) and entity, resolved to the raw form
# by resolve_remote_rpdo()
RPDO_REMOTE_SCHEMA = cv.Schema(
    {
        cv.Required("node"): cv.string,
        cv.Exclusive("entity", "remote_entity"): cv.string,
        cv.Exclusive("index", "remote_entity"): cv.int_,
        cv.Optional("state"): cv.string,
        cv.Optional("cmd", 0): cv.int_,
    }
)


def RPDO_SCHEMA(value):
    if isinstance(value, dict) and "node" in value:
        value = RPDO_REMOTE_SCHEMA(value)
        if "entity" not in value and "index" not in value:
            raise cv.Invalid("'entity' or 'index' is required")
        return value
    return RPDO_RAW_SCHEMA(value)


TPDO_SCHEMA = cv.Schema(
    {
        cv.Required("number"): cv.int_,
        cv.Optional("immediate", default=False): cv.boolean,
    }
)

ENTITY_SCHEMA = cv.Schema(
    {
        cv.Required("id"): cv.use_id(cg.EntityBase),
        cv.Required("index"): cv.int_,
        cv.Optional("size"): cv.int_,
        cv.Optional("min_value"): cv.float_,
        cv.Optional("max_value"): cv.float_,
        cv.Optional("tpdo"): cv.Any(cv.int_, TPDO_SCHEMA),
        cv.Optional("rpdo"): cv.ensure_list(RPDO_SCHEMA),
        # light / cover capabilities, for platforms not known to layout.py
        cv.Optional("caps"): cv.ensure_list(cv.string),
    }
)


def template_entity_cmd_schema(type, trigger):
    return cv.Schema(
        {
            cv.Required("type"): type,
            cv.Required("handler"): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(trigger),
                }
            ),
        }
    )


TEMPLATE_ENTITY_METADATA_SCHEMA = cv.Schema(
    {
        cv.Required("type"): cv.int_,
        cv.Optional("name", ""): cv.string,
        cv.Optional("device_class", ""): cv.string,
        cv.Optional("unit", ""): cv.string,
        cv.Optional("state_class", ""): cv.string,
    }
)

TEMPLATE_ENTITY_STATE_SCHEMA = cv.Schema(
    {
        cv.Required("type"): cv.string,
    }
)

TEMPLATE_ENTITY_CMD_SCHEMA = cv.vol.Union(
    template_entity_cmd_schema("uint8", CmdTriggerUInt8),
    template_entity_cmd_schema("uint16", CmdTriggerUInt16),
    template_entity_cmd_schema("uint32", CmdTriggerUInt32),
    discriminant=lambda val, alt: filter(
        lambda x: x.schema["type"] == val["type"], alt
    ),
)

TEMPLATE_ENTITY = cv.Schema(
    {
        cv.Required("index"): cv.int_,
        cv.Optional("tpdo", -1): cv.int_,
        cv.Optional("commands"): cv.ensure_list(TEMPLATE_ENTITY_CMD_SCHEMA),
        cv.Optional("states"): cv.ensure_list(TEMPLATE_ENTITY_STATE_SCHEMA),
        cv.Optional("metadata"): TEMPLATE_ENTITY_METADATA_SCHEMA,
    }
)

def remote_logs_timeout(value):
    if isinstance(value, str) and value.lower() == "never":
        return "never"
    return cv.All(
        cv.positive_time_period_milliseconds,
        cv.Range(min=cv.TimePeriod(seconds=1), max=cv.TimePeriod(hours=1)),
    )(value)


REMOTE_LOGS_SCHEMA = cv.Schema(
    {
        cv.Optional("buffer_size", default=2048): cv.int_range(min=256, max=16384),
        cv.Optional("level", default="INFO"): cv.one_of(*LOG_LEVEL_SEVERITY, upper=True),
        cv.Optional("timeout", default="30s"): remote_logs_timeout,
        # frames per SDO block of a log upload, advertised in 0x3100:7: the node sends a block back to back,
        # so it must fit the TX queue
        cv.Optional("block_size", default=DEFAULT_LOG_BLOCK_SIZE): cv.int_range(
            min=1, max=127
        ),
    }
)

HB_CLIENT_SCHEMA = cv.Schema(
    {
        cv.Required("node_id"): cv.All(cv.int_, cv.Range(min=0, max=127)),
        cv.Required("timeout"): cv.positive_time_period_milliseconds,
    }
)

CONFIG_SCHEMA = cv.ensure_list(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CanopenComponent),
            cv.GenerateID("canbus_id"): cv.use_id(CanbusComponent),
            # cv.GenerateID("ota_id"): cv.use_id(CanopenOTAComponent),
            cv.Required("node_id"): cv.All(cv.int_, cv.Range(min=0, max=127)),
            # cv.Optional("status"): STATUS_ENTITY_SCHEMA,
            cv.Optional("csdo"): cv.ensure_list(CSDO_SCHEMA),
            # name -> YAML file of another node, see remote.py
            cv.Optional(CONF_REMOTE_NODES): cv.Schema({cv.string: cv.file_}),
            cv.Required(CONF_ENTITIES): cv.ensure_list(ENTITY_SCHEMA),
            cv.Optional("sdo_block_transfer_size", 63): cv.All(
                cv.int_, cv.Range(min=1, max=127)
            ),
            cv.Optional("template_entities"): cv.ensure_list(TEMPLATE_ENTITY),
            cv.Optional("on_pre_operational"): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        PreOperationalTrigger
                    ),
                }
            ),
            cv.Optional("on_operational"): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OperationalTrigger),
                }
            ),
            cv.Optional("on_hb_consumer_event"): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        HbConsumerEventTrigger
                    ),
                }
            ),
            cv.Optional("pdo_od_writer", default=True): cv.boolean,
            cv.Optional(
                "heartbeat_interval", "5000ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional("heartbeat_clients"): cv.ensure_list(HB_CLIENT_SCHEMA),
            cv.Optional("sw_version"): cv.string,
            cv.Optional("hw_version"): cv.string,
            cv.Optional(CONF_REMOTE_LOGS): REMOTE_LOGS_SCHEMA,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    lambda config: resolve_remote_rpdo(config),
)

TYPE_TO_CANOPEN_TYPE = {
    "uint8": (cg.RawExpression("CO_TUNSIGNED8"), 1),
    "uint16": (cg.RawExpression("CO_TUNSIGNED16"), 2),
    "uint32": (cg.RawExpression("CO_TUNSIGNED32"), 4),
    "int8": (cg.RawExpression("CO_TSIGNED8"), 1),
    "int16": (cg.RawExpression("CO_TSIGNED16"), 2),
    "int32": (cg.RawExpression("CO_TSIGNED32"), 4),
}

# entity C++ class -> layout domain; order matters only for classes inheriting from each other
ENTITY_DOMAINS = [
    (cg.esphome_ns.namespace("sensor").class_("Sensor"), layout.DOMAIN_SENSOR),
    (cg.esphome_ns.namespace("number").class_("Number"), layout.DOMAIN_NUMBER),
    (
        cg.esphome_ns.namespace("binary_sensor").class_("BinarySensor"),
        layout.DOMAIN_BINARY_SENSOR,
    ),
    (cg.esphome_ns.namespace("switch_").class_("Switch"), layout.DOMAIN_SWITCH),
    (cg.esphome_ns.namespace("light").class_("LightState"), layout.DOMAIN_LIGHT),
    (cg.esphome_ns.namespace("cover").class_("Cover"), layout.DOMAIN_COVER),
    (
        cg.esphome_ns.namespace("alarm_control_panel").class_("AlarmControlPanel"),
        layout.DOMAIN_ALARM,
    ),
]


CO_RPDO_N = 4  # -DCO_RPDO_N, see to_code()
RPDO_MAX_ENTRIES = 8  # 0x1600+n subindices 1..8 (rpdo_buf in canopen.h)


def resolve_local_entity(full_config, entity_config):
    """(domain, platform config) of a locally exposed entity, via its declared C++ type."""
    entity_id = entity_config["id"]
    for declared_id, path in full_config.declare_ids:
        if declared_id.id == entity_id.id:
            break
    else:
        raise layout.LayoutError(f"ID {entity_id} not found in configuration")
    domain = next(
        (d for cls, d in ENTITY_DOMAINS if declared_id.type.inherits_from(cls)), None
    )
    if domain is None:
        raise layout.LayoutError(
            f"Entity '{entity_id}' of type {declared_id.type} is not supported"
        )
    return domain, full_config.get_config_for_path(path[:-1])


def entity_layout(full_config, entity_config):
    """(caps, [(state name, size)]) of a locally exposed entity, see layout.py."""
    domain, platform_config = resolve_local_entity(full_config, entity_config)
    caps = layout.entity_caps(domain, platform_config, entity_config.get("caps"))
    return caps, layout.entity_states(domain, caps, entity_config.get("size"))


def node_states(full_config, config):
    """layout.node_states() of a local node, with config errors pointing at the entity."""
    try:
        return layout.node_states(
            config, lambda entity: resolve_local_entity(full_config, entity)
        )
    except layout.LayoutError as err:
        path = [CONF_ENTITIES, err.entity_pos] if err.entity_pos is not None else []
        raise cv.Invalid(str(err), path=path) from err


def resolve_remote_rpdo(config):
    """Rewrite `rpdo: {node, entity|index, state, cmd}` entries into the raw form
    `{node_id, tpdo, offset, cmd}` using the remote node's TPDO layout.

    Resolved entries keep `size` (bytes of the remote state) and `remote`
    (dict from RemoteNode.state()).
    """
    remote_nodes = config.get(CONF_REMOTE_NODES, {})
    for entity_pos, entity in enumerate(config.get(CONF_ENTITIES, [])):
        for rpdo_pos, rpdo in enumerate(entity.get(CONF_RPDO, [])):
            if "node" not in rpdo:
                continue
            path = [CONF_ENTITIES, entity_pos, CONF_RPDO, rpdo_pos]
            node = rpdo["node"]
            if node not in remote_nodes:
                raise cv.Invalid(
                    f"Unknown node '{node}', add it to '{CONF_REMOTE_NODES}'",
                    path=path + ["node"],
                )
            try:
                remote_node = remote.get_remote_node(node, remote_nodes[node])
                state = remote_node.state(
                    rpdo.get("entity"), rpdo.get("index"), rpdo.get("state")
                )
            except remote.RemoteError as err:
                raise cv.Invalid(str(err), path=path) from err
            entity[CONF_RPDO][rpdo_pos] = {
                "node_id": state["node_id"],
                "tpdo": state["tpdo"],
                "offset": state["offset"],
                "cmd": rpdo["cmd"],
                "size": state["size"],
                "remote": {**state, "node": node},
            }
    return config


def _walk_remote_states(value):
    """(canopen id, (node, entity, state)) of all validated configs marked with CONF_REMOTE_STATE."""
    if isinstance(value, dict):
        if CONF_REMOTE_STATE in value:
            # platform entities refer to the canopen component by canopen_id, actions by id
            canopen_id = value.get(CONF_CANOPEN_ID, value.get(CONF_ID))
            yield canopen_id, tuple(value[CONF_REMOTE_STATE])
        for item in value.values():
            yield from _walk_remote_states(item)
    elif isinstance(value, list):
        for item in value:
            yield from _walk_remote_states(item)


def remote_state_slots(full_config, canopen_id):
    """Sorted, unique (node, entity, state) remote states mirrored by a canopen component.

    Slot n is the n-th item, OD 0x4000 + n. Computed from the full validated config, so
    the canopen component, the `platform: canopen` entities and the actions agree on it.
    """
    return sorted(
        {
            key
            for parent, key in _walk_remote_states(dict(full_config))
            if parent is not None and str(parent) == str(canopen_id)
        }
    )


def remote_state_slot(full_config, canopen_id, key):
    return remote_state_slots(full_config, canopen_id).index(tuple(key))


def resolve_remote_slots(full_config, config):
    """[(slot n, RemoteNode.state() + node)] of a canopen node config."""
    remote_nodes = config.get(CONF_REMOTE_NODES, {})
    slots = []
    for n, (node, entity, state) in enumerate(
        remote_state_slots(full_config, config[CONF_ID])
    ):
        if node not in remote_nodes:
            raise cv.Invalid(
                f"Unknown node '{node}' (used by {entity}.{state}), add it to '{CONF_REMOTE_NODES}'"
            )
        try:
            remote_node = remote.get_remote_node(node, remote_nodes[node])
            info = remote_node.state(entity, None, state)
        except remote.RemoteError as err:
            raise cv.Invalid(str(err)) from err
        slots.append((n, {**info, "node": node}))
    return slots


def rpdo_plan(config, slots=()):
    """RPDOs of a node: [(node_id, tpdo, [("dummy", size) | ("cmd", entity_index, cmd, size)
    | ("slot", n, size)])].

    One RPDO per remote (node_id, tpdo); entries sorted by offset, gaps filled with
    dummy entries. Sizes in bytes; raw-form entries map 1 byte. `slots` as returned
    by resolve_remote_slots().
    """
    rpdos = sorted(
        [
            {**rpdo, "entity_index": entity["index"], "size": rpdo.get("size", 1)}
            for entity in config.get(CONF_ENTITIES, [])
            for rpdo in entity.get(CONF_RPDO, ())
        ]
        + [{**state, "slot": n} for n, state in slots],
        key=lambda rpdo: (rpdo["node_id"], rpdo["tpdo"], rpdo["offset"]),
    )
    plan = []
    for (node_id, tpdo), group in groupby(
        rpdos, key=lambda rpdo: (rpdo["node_id"], rpdo["tpdo"])
    ):
        entries = []
        curr_offs = 0
        prev = None
        for rpdo in group:
            if rpdo["offset"] < curr_offs:
                if "slot" in rpdo or "slot" in prev:
                    slot = rpdo if "slot" in rpdo else prev
                    other = prev if slot is rpdo else rpdo
                    raise cv.Invalid(
                        f"{slot['node']} {slot['entity']}.{slot['state']} is mapped by the rpdo of entity "
                        f"{other['entity_index']} and also needed by a 'platform: canopen' entity or a "
                        f"toggle action: replace that entity with 'platform: canopen'"
                    )
                raise cv.Invalid(
                    f"RPDO from node 0x{node_id:02x} TPDO{tpdo}: entity "
                    f"{rpdo['entity_index']} at offset {rpdo['offset']} overlaps the previous entry"
                )
            if rpdo["offset"] > curr_offs:
                entries.append(("dummy", rpdo["offset"] - curr_offs))
            if "slot" in rpdo:
                entries.append(("slot", rpdo["slot"], rpdo["size"]))
            else:
                entries.append(("cmd", rpdo["entity_index"], rpdo["cmd"], rpdo["size"]))
            curr_offs = rpdo["offset"] + rpdo["size"]
            prev = rpdo
        plan.append((node_id, tpdo, entries))
    return plan


def rpdo_dummy_entries(size):
    """Number of mapping entries CanopenComponent::add_rpdo_dummy() uses for `size` bytes."""
    return size // 4 + {0: 0, 1: 1, 2: 1, 3: 2}[size % 4]


def validate_rpdo_plan(plan):
    if len(plan) > CO_RPDO_N:
        sources = ", ".join(f"node 0x{n:02x} TPDO{t}" for n, t, _ in plan)
        raise cv.Invalid(
            f"Too many RPDOs: {len(plan)} (max {CO_RPDO_N}, one per remote node TPDO): {sources}"
        )
    for node_id, tpdo, entries in plan:
        count = sum(
            rpdo_dummy_entries(entry[1]) if entry[0] == "dummy" else 1
            for entry in entries
        )
        if count > RPDO_MAX_ENTRIES:
            raise cv.Invalid(
                f"RPDO from node 0x{node_id:02x} TPDO{tpdo} needs {count} mapping entries "
                f"(max {RPDO_MAX_ENTRIES})"
            )


def validate_remote_rpdo_sizes(full_config, config):
    """A remote state must have the size of the local command it's mapped to."""
    for entity_pos, entity in enumerate(config.get(CONF_ENTITIES, [])):
        local_states = None
        for rpdo_pos, rpdo in enumerate(entity.get(CONF_RPDO, ())):
            if "remote" not in rpdo:
                continue
            if local_states is None:
                local_states = entity_layout(full_config, entity)[1]
            path = [CONF_ENTITIES, entity_pos, CONF_RPDO, rpdo_pos]
            if rpdo["cmd"] >= len(local_states):
                raise cv.Invalid(
                    f"'{entity['id']}' has no command {rpdo['cmd']}", path=path
                )
            local_name, local_size = local_states[rpdo["cmd"]]
            if local_size != rpdo["size"]:
                src = rpdo["remote"]
                raise cv.Invalid(
                    f"{src['node']} {src['entity']}.{src['state']} is {rpdo['size']} B, but "
                    f"'{entity['id']}' command {rpdo['cmd']} ({local_name}) is {local_size} B",
                    path=path,
                )


def validate_remote_entities(full_config, config):
    """`platform: canopen` entities may be exposed (for debugging), but get their state from a slot."""
    remote_ids = {
        str(value[CONF_ID])
        for domain in ("binary_sensor", "sensor", "text_sensor")
        for value in full_config.get(domain) or ()
        if value.get("platform") == "canopen"
    }
    for entity_pos, entity in enumerate(config.get(CONF_ENTITIES, [])):
        if str(entity["id"]) in remote_ids and entity.get(CONF_RPDO):
            raise cv.Invalid(
                f"'{entity['id']}' is a 'platform: canopen' entity, it can't have 'rpdo'",
                path=[CONF_ENTITIES, entity_pos, CONF_RPDO],
            )


def validate_remote_logs(full_config, config_list):
    with_logs = [config for config in config_list if CONF_REMOTE_LOGS in config]
    if len(with_logs) > 1:
        raise cv.Invalid(f"'{CONF_REMOTE_LOGS}' can be set on one canopen node only")
    for config in with_logs:
        remote_logs = config[CONF_REMOTE_LOGS]
        if "logger" not in full_config:
            raise cv.Invalid(
                f"'{CONF_REMOTE_LOGS}' requires the 'logger' component",
                path=[CONF_REMOTE_LOGS],
            )
        if remote_logs["block_size"] > config["sdo_block_transfer_size"]:
            # upload blocks go through the SDO buffer, canopen-stack caps a larger request without telling
            # the host
            raise cv.Invalid(
                f"block_size ({remote_logs['block_size']}) can't be larger than "
                f"sdo_block_transfer_size ({config['sdo_block_transfer_size']})",
                path=[CONF_REMOTE_LOGS, "block_size"],
            )
        level = remote_logs["level"]
        logger_level = full_config["logger"].get("level", "DEBUG")
        if LOG_LEVEL_SEVERITY.index(level) > LOG_LEVEL_SEVERITY.index(logger_level):
            _LOGGER.warning(
                "%s: level %s is more verbose than logger level %s, "
                "those messages are compiled out",
                CONF_REMOTE_LOGS,
                level,
                logger_level,
            )
        if remote_logs["timeout"] == "never" and level == "NONE":
            _LOGGER.warning(
                "%s: 'timeout: never' with 'level: NONE' never expires a host "
                "subscription",
                CONF_REMOTE_LOGS,
            )


def _final_validate(config_list):
    full_config = fv.full_config.get()
    validate_remote_logs(full_config, config_list)
    for config in config_list:
        try:
            layout.tpdo_mapping(node_states(full_config, config))
        except layout.LayoutError as err:
            raise cv.Invalid(str(err)) from err
        validate_remote_entities(full_config, config)
        validate_remote_rpdo_sizes(full_config, config)
        validate_rpdo_plan(rpdo_plan(config, resolve_remote_slots(full_config, config)))
    return config_list


FINAL_VALIDATE_SCHEMA = _final_validate


def to_code(config_list):
    # No compression on stm32 or zephyr: miniz needs ~43 KB of heap for inflate
    if not (getattr(CORE, "is_stm32", False) or getattr(CORE, "is_zephyr", False)):
        extra_build_flags = ("-DOTA_COMPRESSION=1", )
    else:
        extra_build_flags = ()

    for config in config_list:
        for flag in [
            "-DCO_SDO_BUF_SEG={}".format(config["sdo_block_transfer_size"]),
            "-DCO_SSDO_N=1",
            "-DCO_CSDO_N=1",
            "-DCO_RPDO_N=4",
            "-DCO_TPDO_N=8",
            "-DUSE_LSS=0",
            "-DUSE_CSDO=0",
            "-DMINIZ_NO_STDIO=1",
            "-DMINIZ_NO_TIME=1",
            *extra_build_flags,
        ]:
            cg.add_build_flag(flag)

        cg.add_library(
            "canopenstack", None, "https://github.com/mrk-its/canopen-stack#test"
        )
        node_id = config["node_id"]
        canopen = cg.new_Pvariable(config[CONF_ID], node_id)

        canbus = yield cg.get_variable(config["canbus_id"])
        cg.add(canopen.set_canbus(canbus))

        cg.add(canopen.set_heartbeat_interval(config["heartbeat_interval"]))
        cg.add(canopen.enable_pdo_od_writer(config["pdo_od_writer"]))
        hw_version = config.get("hw_version")
        sw_version = config.get("sw_version")

        ts = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d.%H%M%S")

        cg.add(
            cg.RawStatement(
                f"""const char * version_str_{node_id} = "node_id: {node_id:02x}, ver: " ESPHOME_VERSION ".{ts}";"""
            )
        )

        sw_version = f"version_str_{node_id} + 18"

        if hw_version:
            cg.add(canopen.od_set_string(0x1009, 0, hw_version))
        if sw_version:
            cg.add(canopen.od_set_string(0x100A, 0, cg.RawExpression(sw_version)))

        yield cg.register_component(canopen, config)

        remote_logs = config.get(CONF_REMOTE_LOGS)
        if remote_logs is not None:
            try:
                from esphome.components.logger import request_log_listener
            except ImportError:  # older ESPHome: log listeners always enabled
                pass
            else:
                request_log_listener()
            cg.add_define("USE_CANOPEN_LOGS")
            cg.add_define("CANOPEN_LOG_BUFFER_SIZE", remote_logs["buffer_size"])
            timeout = remote_logs["timeout"]
            cg.add(
                canopen.set_log_default_level(
                    LOG_LEVEL_SEVERITY.index(remote_logs["level"])
                )
            )
            cg.add(
                canopen.set_log_timeout(
                    0 if timeout == "never" else timeout.total_milliseconds
                )
            )
            cg.add(canopen.set_log_block_size(remote_logs["block_size"]))
            # register the log callback now, not in setup(): codegen runs before App.setup(), so the setup()
            # logs of components with a higher setup priority are captured too
            cg.add(canopen.start_logs())

        entities = sorted(config.get(CONF_ENTITIES, []), key=lambda x: x["index"])
        assert len(entities) == len(
            set(e["index"] for e in entities)
        ), "All entity indices must be unique!"
        states = node_states(CORE.config, config)
        tpdo_layout_hash = layout.layout_hash(layout.tpdo_mapping(states))
        cg.add(canopen.set_expected_tpdo_layout_hash(tpdo_layout_hash))
        _LOGGER.info(
            "CANopen node 0x%02x TPDO layout (hash %08x):%s",
            node_id,
            tpdo_layout_hash,
            "".join(f"\n  {line}" for line in layout.layout_table(states, node_id))
            or " no TPDOs",
        )

        for entity_config in entities:
            entity = yield cg.get_variable(entity_config["id"])
            caps, _ = entity_layout(CORE.config, entity_config)
            tpdo = entity_config.get("tpdo", -1)
            if not isinstance(tpdo, dict):
                tpdo = {"number": tpdo, "immediate": False}

            tpdo_struct = cg.StructInitializer(
                TPDO, ("number", tpdo["number"]), ("immediate", tpdo["immediate"])
            )

            size = entity_config.get("size")
            if size in (1, 2):
                min_val = entity_config.get("min_value", 0)
                max_val = entity_config.get(
                    "max_value", 254 if size == 1 else 65534
                )  # 255 / 65535 reserved for NaN
                cg.add(
                    canopen.add_entity(
                        entity,
                        entity_config["index"],
                        tpdo_struct,
                        size,
                        min_val,
                        max_val,
                    )
                )
            elif caps is not None:
                cg.add(
                    canopen.add_entity(
                        entity, entity_config["index"], tpdo_struct, caps
                    )
                )
            else:
                cg.add(canopen.add_entity(entity, entity_config["index"], tpdo_struct))

        for tmpl_entity in config.get("template_entities", []):
            index = tmpl_entity["index"]
            metadata = tmpl_entity.get("metadata")
            if metadata:
                cg.add(
                    canopen.od_add_metadata(
                        index,
                        metadata["type"],
                        metadata["name"],
                        metadata["device_class"],
                        metadata["unit"],
                        metadata["state_class"],
                    )
                )
            for state in tmpl_entity.get("states", ()):
                _type, size = TYPE_TO_CANOPEN_TYPE[state["type"]]
                cg.add(canopen.od_add_state(index, _type, 0, size, tmpl_entity["tpdo"]))
                pass
            for cmd in tmpl_entity.get("commands", ()):
                for handler in cmd["handler"]:
                    trigger = cg.new_Pvariable(handler[CONF_TRIGGER_ID])
                    yield automation.build_automation(
                        trigger,
                        [
                            (getattr(cg, cmd["type"]), "x"),
                        ],
                        handler,
                    )
                    cg.add(
                        canopen.add_entity_cmd(
                            index, tmpl_entity.get("tpdo", -1), trigger
                        )
                    )

        for num, csdo in enumerate(config.get("csdo", ())):
            cg.add(
                canopen.setup_csdo(num, csdo["node_id"], csdo["tx_id"], csdo["rx_id"])
            )
        for num, client in enumerate(config.get("heartbeat_clients", ()), 1):
            cg.add(
                canopen.setup_heartbeat_client(
                    num, client["node_id"], client["timeout"]
                )
            )

        for conf in config.get("on_operational", []):
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
            yield automation.build_automation(trigger, [], conf)
            cg.add(canopen.add_trigger(trigger))

        for conf in config.get("on_pre_operational", []):
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
            yield automation.build_automation(trigger, [], conf)
            cg.add(canopen.add_trigger(trigger))

        for conf in config.get("on_hb_consumer_event", []):
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
            yield automation.build_automation(
                trigger,
                [
                    (cg.uint8, "node_id"),
                ],
                conf,
            )
            cg.add(canopen.add_trigger(trigger))

        slots = resolve_remote_slots(CORE.config, config)
        for n, state in slots:
            _LOGGER.info(
                "CANopen node 0x%02x remote state slot %d (0x%04x): %s %s.%s",
                config["node_id"],
                n,
                REMOTE_STATE_INDEX + n,
                state["node"],
                state["entity"],
                state["state"],
            )
            cg.add(canopen.add_remote_state(n, state["size"]))

        for idx, (node_id, tpdo, entries) in enumerate(rpdo_plan(config, slots)):
            cg.add(canopen.add_rpdo_node(idx, node_id, tpdo))
            for entry in entries:
                if entry[0] == "dummy":
                    cg.add(canopen.add_rpdo_dummy(idx, entry[1]))
                elif entry[0] == "slot":
                    cg.add(canopen.add_rpdo_slot(idx, entry[1], entry[2] * 8))
                else:
                    _, entity_index, cmd, size = entry
                    cg.add(
                        canopen.add_rpdo_entity_cmd(idx, entity_index, cmd, size * 8)
                    )


def _raw_canopen_configs():
    """`canopen:` node configs of the local raw config (packages and substitutions
    applied, not validated yet), available while actions are validated."""
    canopen = (CORE.raw_config or {}).get("canopen") or []
    return canopen if isinstance(canopen, list) else [canopen]


def _raw_remote_node(node):
    for canopen in _raw_canopen_configs():
        path = (canopen.get(CONF_REMOTE_NODES) or {}).get(node)
        if path is not None:
            return remote.get_remote_node(node, CORE.relative_config_path(path))
    raise cv.Invalid(f"Unknown node '{node}', add it to canopen '{CONF_REMOTE_NODES}'")


def _resolve_action_target(config):
    """Set `node_id` / `index` from `node` + `entity` / `index` (remote_nodes)."""
    if "node" not in config:
        if "node_id" not in config or "index" not in config:
            raise cv.Invalid("either 'node' + 'entity' / 'index', or 'node_id' + 'index' is required")
        return config
    if "entity" not in config and "index" not in config:
        raise cv.Invalid("'entity' or 'index' is required")
    remote_node = _raw_remote_node(config["node"])
    try:
        entity = remote_node.entity(config.get("entity"), config.get("index"))
    except remote.RemoteError as err:
        raise cv.Invalid(str(err)) from err
    return {**config, "node_id": remote_node.node_id, "index": entity["index"]}


ACTION_TARGET_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(CanopenComponent),
        cv.Exclusive("node", "remote_node"): cv.string,
        cv.Exclusive("node_id", "remote_node"): cv.int_range(min=1, max=127),
        cv.Exclusive("entity", "remote_entity"): cv.string,
        cv.Exclusive("index", "remote_entity"): cv.int_,
    }
)


@automation.register_action(
    "canopen.entity_cmd",
    EntityCmdAction,
    cv.All(
        ACTION_TARGET_SCHEMA.extend(
            {
                cv.Optional("cmd", 0): cv.int_range(min=0, max=254),
                cv.Required("value"): cv.templatable(cv.uint8_t),
            }
        ),
        _resolve_action_target,
    ),
    synchronous=True,
)
async def entity_cmd_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_target(config["node_id"], config["index"], config["cmd"]))
    cg.add(var.set_value(await cg.templatable(config["value"], args, cg.uint8)))
    return var


# RemoteCommandAction write encodings, see RemoteCmdEncoding in canopen.h
ENC_RAW = 0
ENC_PERCENTAGE = 1
ENC_COLOR_TEMP = 2
ENC_SCALED = 3
ENC_REAL32 = 4

# value of a toggle write: C++ expression over the raw remote state `s` (slot value)
TOGGLE_BOOL = ("toggle", "s ? 0.0f : 1.0f")
# moving (1 opening, 3 closing) -> stop (0); closed (2) -> open (1); open (0) -> close (2)
TOGGLE_COVER = ("toggle", "(s == 1 || s == 3) ? 0.0f : (s == 2 ? 1.0f : 2.0f)")

REMOTE_ACTION_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(CanopenComponent),
        cv.Required("node"): cv.string,
        cv.Exclusive("entity", "remote_entity"): cv.string,
        cv.Exclusive("index", "remote_entity"): cv.int_,
    }
)


def _write(cmd, value, size=1, encoding=ENC_RAW, min_value=0.0, max_value=0.0):
    return {
        "cmd": cmd,
        "size": size,
        "encoding": encoding,
        "min_value": min_value,
        "max_value": max_value,
        "value": value,
    }


class _Target:
    """Remote entity an action is validated against."""

    def __init__(self, action, config):
        self.action = action
        self.node = _raw_remote_node(config["node"])
        try:
            self.entity = self.node.entity(config.get("entity"), config.get("index"))
        except remote.RemoteError as err:
            raise cv.Invalid(str(err)) from err
        self.domain = self.node.domain(index=self.entity["index"])
        self.states = self.node.state_names(index=self.entity["index"])

    def cmd(self, state):
        """Command index of a remote state (states and commands are in the same order)."""
        if state not in self.states:
            raise cv.Invalid(
                f"{self.action}: {self.node.name} {self.domain} '{self.entity['id']}' has no "
                f"{state} (states: {', '.join(self.states)})"
            )
        return self.states.index(state)

    def check_state(self, state):
        """The remote state must be sent in a TPDO to be read (toggles)."""
        try:
            self.node.state(index=self.entity["index"], state=state)
        except remote.RemoteError as err:
            raise cv.Invalid(f"{self.action}: {err}") from err


def _remote_action(name, domains, writes, schema=None, toggle=False):
    """Register `canopen.<domain>.<action>`, sending the commands `writes(config, target)` returns."""

    def validate(config):
        if "entity" not in config and "index" not in config:
            raise cv.Invalid("'entity' or 'index' is required")
        target = _Target(name, config)
        if target.domain not in domains:
            raise cv.Invalid(
                f"{name}: {target.node.name} '{target.entity['id']}' is a {target.domain}, "
                f"expected {' or '.join(domains)}"
            )
        config = {
            **config,
            "entity": str(target.entity["id"]),
            "node_id": target.node.node_id,
            "index": target.entity["index"],
            "_writes": writes(config, target),
        }
        if toggle:
            target.check_state("state")
            config[CONF_REMOTE_STATE] = [config["node"], config["entity"], "state"]
        return config

    @automation.register_action(
        name,
        RemoteCommandAction,
        cv.All(REMOTE_ACTION_SCHEMA.extend(schema or {}), validate),
        synchronous=True,
    )
    async def action_to_code(config, action_id, template_arg, args):
        var = cg.new_Pvariable(action_id, template_arg)
        await cg.register_parented(var, config[CONF_ID])
        cg.add(var.set_target(config["node_id"], config["index"]))
        for write in config["_writes"]:
            value = write["value"]
            if isinstance(value, tuple) and value[0] == "toggle":
                n = remote_state_slot(
                    CORE.config, config[CONF_ID], config[CONF_REMOTE_STATE]
                )
                value = await cg.process_lambda(
                    Lambda(
                        f"uint32_t s = id({config[CONF_ID]}).get_remote_state({n}); return {value[1]};"
                    ),
                    args,
                    return_type=cg.float_,
                )
            else:
                value = await cg.templatable(value, args, cg.float_)
            cg.add(
                var.add_write(
                    write["cmd"],
                    write["size"],
                    write["encoding"],
                    write["min_value"],
                    write["max_value"],
                    value,
                )
            )
        return var

    return action_to_code


def _const_writes(*values):
    return lambda config, target: [_write(0, value) for value in values]


def _light_turn_on_writes(config, target):
    writes = []
    if "brightness" in config:
        writes.append(
            _write(target.cmd("brightness"), config["brightness"], encoding=ENC_PERCENTAGE)
        )
    if "color_temperature" in config:
        writes.append(
            _write(
                target.cmd("color_temp"), config["color_temperature"], encoding=ENC_COLOR_TEMP
            )
        )
    # brightness alone doesn't switch an ESPHome light on
    return writes + [_write(0, 1)]


def _cover_control_writes(config, target):
    writes = []
    if config.get("stop"):
        writes.append(_write(0, 0))
    for option in ("position", "tilt"):
        if option in config:
            writes.append(_write(target.cmd(option), config[option], encoding=ENC_PERCENTAGE))
    if not writes:
        raise cv.Invalid("canopen.cover.control: set 'stop', 'position' or 'tilt'")
    return writes


def _number_set_writes(config, target):
    size = target.entity.get("size", 4)
    if size == 4:
        return [_write(0, config["value"], size=4, encoding=ENC_REAL32)]
    min_value, max_value = target.node.scaling(target.entity)
    return [_write(0, config["value"], size, ENC_SCALED, min_value, max_value)]


_LIGHT_SWITCH = (layout.DOMAIN_LIGHT, layout.DOMAIN_SWITCH)
_SWITCH_LIGHT = (layout.DOMAIN_SWITCH, layout.DOMAIN_LIGHT)

_remote_action(
    "canopen.light.turn_on",
    _LIGHT_SWITCH,
    _light_turn_on_writes,
    {
        cv.Optional("brightness"): cv.templatable(cv.percentage),
        cv.Optional("color_temperature"): cv.templatable(cv.color_temperature),
    },
)
_remote_action("canopen.light.turn_off", _LIGHT_SWITCH, _const_writes(0))
_remote_action("canopen.light.toggle", _LIGHT_SWITCH, _const_writes(TOGGLE_BOOL), toggle=True)
_remote_action("canopen.switch.turn_on", _SWITCH_LIGHT, _const_writes(1))
_remote_action("canopen.switch.turn_off", _SWITCH_LIGHT, _const_writes(0))
_remote_action("canopen.switch.toggle", _SWITCH_LIGHT, _const_writes(TOGGLE_BOOL), toggle=True)

_COVER = (layout.DOMAIN_COVER,)
_remote_action("canopen.cover.open", _COVER, _const_writes(1))
_remote_action("canopen.cover.close", _COVER, _const_writes(2))
_remote_action("canopen.cover.stop", _COVER, _const_writes(0))
_remote_action("canopen.cover.toggle", _COVER, _const_writes(TOGGLE_COVER), toggle=True)
_remote_action(
    "canopen.cover.control",
    _COVER,
    _cover_control_writes,
    {
        cv.Optional("stop"): cv.boolean,
        cv.Optional("position"): cv.templatable(cv.percentage),
        cv.Optional("tilt"): cv.templatable(cv.percentage),
    },
)

# alarm command codes, see Alarm Control Panel in OBJECT_DICTIONARY.md
for _action, _code in (
    ("disarm", 0),
    ("arm_away", 1),
    ("arm_home", 2),
    ("arm_night", 3),
    ("arm_vacation", 4),
    ("arm_custom_bypass", 5),
):
    _remote_action(
        f"canopen.alarm_control_panel.{_action}",
        (layout.DOMAIN_ALARM,),
        _const_writes(_code),
    )

_remote_action(
    "canopen.number.set",
    (layout.DOMAIN_NUMBER,),
    _number_set_writes,
    {cv.Required("value"): cv.templatable(cv.float_)},
)


# `platform: canopen` binary_sensor / sensor / text_sensor (binary_sensor.py, ...)
REMOTE_STATE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CANOPEN_ID): cv.use_id(CanopenComponent),
        cv.Required("node"): cv.string,
        cv.Required("entity"): cv.string,
        cv.Optional("state"): cv.string,
    }
)


def validate_remote_state(platform, kinds):
    """Resolve the remote state of a `platform: canopen` entity and check its kind."""

    def validator(config):
        remote_node = _raw_remote_node(config["node"])
        try:
            state = remote_node.state(config["entity"], None, config.get("state"))
        except remote.RemoteError as err:
            raise cv.Invalid(str(err)) from err
        if state["kind"] not in kinds:
            raise cv.Invalid(
                f"{platform} can't show {config['node']} {state['entity']}.{state['state']} "
                f"({state['kind']}), supported: {', '.join(kinds)}"
            )
        return {
            **config,
            "state": state["state"],
            "_remote": state,
            CONF_REMOTE_STATE: [config["node"], state["entity"], state["state"]],
        }

    return validator


async def attach_remote_state(var, config):
    """Subscribe a `platform: canopen` entity to its remote state slot."""
    parent = await cg.get_variable(config[CONF_CANOPEN_ID])
    n = remote_state_slot(CORE.config, config[CONF_CANOPEN_ID], config[CONF_REMOTE_STATE])
    state = config["_remote"]
    cg.add(
        var.attach(
            parent,
            n,
            layout.KIND_CODES[state["kind"]],
            state["size"],
            state.get("min_value", 0.0),
            state.get("max_value", 0.0),
        )
    )

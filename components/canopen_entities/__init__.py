"""ESPHome entities in a CANopen object dictionary, for can2mqtt (OBJECT_DICTIONARY.md).

Sits on top of the `canopen` hub (`canopen_id`): entity metadata, states and commands at 0x2000 + 16 * N, TPDOs
computed by layout.py, the layout hash at 0x3002, the ESPHome identity can2mqtt checks, the OD writer, and links to
other nodes (`remote_nodes`, docs/linking-nodes.md): remote state slots at 0x4000 + n and entity commands filled by
RPDOs from the other nodes' TPDOs, `platform: canopen_entities` entities and `canopen_entities.<domain>.<action>` actions.
"""

from collections.abc import Iterable
import datetime as dt
import logging
from typing import Any

from esphome import automation
import esphome.codegen as cg
from esphome.components import canopen
from esphome.components.logger import LOG_LEVEL_SEVERITY, request_log_listener
import esphome.config_validation as cv
from esphome.const import (
    CONF_BUFFER_SIZE,
    CONF_ID,
    CONF_INDEX,
    CONF_LEVEL,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_NUMBER,
    CONF_OFFSET,
    CONF_SIZE,
    CONF_STATE,
    CONF_TIMEOUT,
)
from esphome.core import CORE, Lambda
from esphome.cpp_generator import MockObj
import esphome.final_validate as fv
from esphome.schema_extractors import SCHEMA_EXTRACT, schema_extractor
from esphome.types import ConfigType

from . import layout, remote

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@mrk-its"]
DEPENDENCIES = ["canopen"]

CONF_ENTITIES = "entities"
CONF_TPDO = "tpdo"
CONF_IMMEDIATE = "immediate"
CONF_CAPS = "caps"
CONF_OD_WRITER = "od_writer"
CONF_REMOTE_NODES = "remote_nodes"
CONF_RPDO = "rpdo"
CONF_NODE = "node"
CONF_ENTITY = "entity"
CONF_CMD = "cmd"
CONF_TPDO_SIZE = "tpdo_size"
CONF_MAPPER_ID = "canopen_entities_id"
CONF_REMOTE_LOGS = "remote_logs"
CONF_BLOCK_SIZE = "block_size"
# remote state (node, entity, state) a `platform: canopen_entities` entity or a toggle action reads, set by
# their validation; collected from the whole validated config by remote_state_slots()
CONF_REMOTE_STATE = "_canopen_remote_state"
# the OD writer listens on 0x500..0x57F, which are the TPDO 7 COB-IDs (0x500 + node) of all nodes
OD_WRITER_TPDO = 7

# what can2mqtt accepts (can2mqtt.py ESPHOME_VENDOR_ID / ESPHOME_PRODUCT_CODE); revision 1: entity types in 0x2001
ESPHOME_VENDOR_ID = 0xA59A08F5
ESPHOME_PRODUCT_CODE = 0x6BDFA1D9
ESPHOME_REVISION = 1

# SDO block size (download cap) when the hub doesn't set sdo_block_size, for OTA and remote logs: a download block
# arrives back to back, so it must fit the CAN receive queue (e.g. zephyr_can rx_buffer_size)
DEFAULT_SDO_BLOCK_SIZE = 16
# frames per Log Data upload block without a software TX queue: the 3 TX mailboxes of bxCAN / FDCAN
DEFAULT_LOG_BLOCK_SIZE = 3
LOG_TIMEOUT_NEVER = "never"

ns = cg.esphome_ns.namespace("canopen_entities")
CANopenEntities = ns.class_("CANopenEntities", cg.Component)
RemoteCommandAction = ns.class_("RemoteCommandAction", automation.Action)
RemoteLogs = ns.class_("RemoteLogs", cg.Component)

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

TPDO_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_NUMBER): cv.int_range(min=0, max=layout.TPDO_COUNT - 1),
        # kept for the old YAML: every change is sent at the next CANopen processing either way
        cv.Optional(CONF_IMMEDIATE, default=False): cv.boolean,
    }
)

# a remote node's entity state, by node name (remote_nodes) and entity, into command `cmd` of the local entity
RPDO_REMOTE_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Required(CONF_NODE): cv.string,
            cv.Exclusive(CONF_ENTITY, "remote_entity"): cv.string,
            cv.Exclusive(CONF_INDEX, "remote_entity"): cv.int_,
            cv.Optional(CONF_STATE): cv.string,
            cv.Optional(CONF_CMD, default=0): cv.int_range(min=0, max=254),
        }
    ),
    cv.has_at_least_one_key(CONF_ENTITY, CONF_INDEX),
)


def validate_raw_rpdo_tpdo_size(config: ConfigType) -> ConfigType:
    tpdo_size = config.get(CONF_TPDO_SIZE)
    if tpdo_size is not None and tpdo_size <= config[CONF_OFFSET]:
        raise cv.Invalid(
            f"offset {config[CONF_OFFSET]} is outside a {tpdo_size} B TPDO",
            path=[CONF_OFFSET],
        )
    return config


# raw TPDO position: one byte at `offset` of TPDO `tpdo` of node `node_id`, into command `cmd` of the local entity;
# `tpdo_size`: the length of that TPDO (the RPDO maps it all, unset: up to the last mapped byte)
RPDO_RAW_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Required(canopen.CONF_NODE_ID): cv.int_range(min=1, max=127),
            cv.Required(CONF_TPDO): cv.int_range(min=0, max=layout.TPDO_COUNT - 1),
            cv.Required(CONF_OFFSET): cv.int_range(min=0, max=7),
            cv.Optional(CONF_CMD, default=0): cv.int_range(min=0, max=254),
            cv.Optional(CONF_TPDO_SIZE): cv.int_range(min=1, max=8),
        }
    ),
    validate_raw_rpdo_tpdo_size,
)


@schema_extractor("schema")
def validate_rpdo(value: Any) -> ConfigType:
    """The raw form if `node_id` is given, else the named one (picked here, not by cv.Any: errors of the right form)."""
    if value is SCHEMA_EXTRACT:
        return cv.Any(RPDO_RAW_SCHEMA, RPDO_REMOTE_SCHEMA)
    if isinstance(value, dict) and canopen.CONF_NODE_ID in value:
        return RPDO_RAW_SCHEMA(value)
    return RPDO_REMOTE_SCHEMA(value)


ENTITY_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(cg.EntityBase),
        # 0x2000 + 16 * index; 0 is can2mqtt's NMT state entity
        cv.Required(CONF_INDEX): cv.int_range(min=1, max=255),
        cv.Optional(CONF_SIZE): cv.one_of(1, 2, 4, int=True),
        cv.Optional(CONF_MIN_VALUE): cv.float_,
        cv.Optional(CONF_MAX_VALUE): cv.float_,
        cv.Optional(CONF_TPDO): cv.Any(
            cv.int_range(min=0, max=layout.TPDO_COUNT - 1), TPDO_SCHEMA
        ),
        # light / cover capabilities, for platforms not known to layout.py
        cv.Optional(CONF_CAPS): cv.ensure_list(cv.string),
        # remote states written into this entity's commands by RPDO (resolve_remote_rpdo())
        cv.Optional(CONF_RPDO): cv.ensure_list(validate_rpdo),
    }
)


# remote logs (OD 0x3100, docs/remote-logs.md)
REMOTE_LOGS_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RemoteLogs),
        cv.Optional(CONF_BUFFER_SIZE, default=2048): cv.int_range(min=256, max=16384),
        # the level from boot until a host subscribes
        cv.Optional(CONF_LEVEL, default="INFO"): cv.one_of(
            *LOG_LEVEL_SEVERITY, upper=True
        ),
        # logging turns off without a host poll for this long
        cv.Optional(CONF_TIMEOUT, default="30s"): cv.Any(
            cv.one_of(LOG_TIMEOUT_NEVER, lower=True),
            cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(seconds=1), max=cv.TimePeriod(hours=1)),
            ),
        ),
        # frames per SDO block of a Log Data upload, advertised in 0x3100:7: the node sends a block back to back, so
        # it must fit the TX queue
        cv.Optional(CONF_BLOCK_SIZE, default=DEFAULT_LOG_BLOCK_SIZE): cv.int_range(
            min=1, max=127
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


# what canopen_entities needs from the hub beyond its first upstream PR: own PDOs (PR 5), SDO block transfer (PR 6)
HUB_REQUIRED = ("enable_pdo", "enable_sdo_block", "CONF_SDO_BLOCK_SIZE")
HUB_SOURCE = "github://mrk-its/esphome@canopen-full"


def validate_hub(config: ConfigType) -> ConfigType:
    """A `canopen` hub with everything canopen_entities uses (ESPHome may ship a partial one while the PRs go in)."""
    if missing := [name for name in HUB_REQUIRED if not hasattr(canopen, name)]:
        raise cv.Invalid(
            f"this ESPHome's 'canopen' component lacks {', '.join(missing)}, which canopen_entities needs: "
            f"load the full hub as an external component (source: {HUB_SOURCE}, components: [canopen])"
        )
    return config


def validate_unique_indices(config: ConfigType) -> ConfigType:
    seen = set()
    for pos, entity in enumerate(config[CONF_ENTITIES]):
        if entity[CONF_INDEX] in seen:
            raise cv.Invalid(
                f"index {entity[CONF_INDEX]} is used twice",
                path=[CONF_ENTITIES, pos, CONF_INDEX],
            )
        seen.add(entity[CONF_INDEX])
    return config


def validate_od_writer_tpdo(config: ConfigType) -> ConfigType:
    if not config[CONF_OD_WRITER]:
        return config
    for pos, entity in enumerate(config[CONF_ENTITIES]):
        if CONF_TPDO in entity and layout.tpdo_number(entity) == OD_WRITER_TPDO:
            raise cv.Invalid(
                f"TPDO {OD_WRITER_TPDO} is reserved for the OD writer (od_writer: false frees it)",
                path=[CONF_ENTITIES, pos, CONF_TPDO],
            )
    return config


CONFIG_SCHEMA = cv.All(
    validate_hub,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CANopenEntities),
            cv.GenerateID(canopen.CONF_CANOPEN_ID): cv.use_id(canopen.CANopenComponent),
            cv.Optional(CONF_ENTITIES, default=[]): cv.ensure_list(ENTITY_SCHEMA),
            # node-to-node commands (OBJECT_DICTIONARY.md "OD writer"); reserves TPDO 7
            cv.Optional(CONF_OD_WRITER, default=True): cv.boolean,
            # name -> YAML file of another node, see remote.py
            cv.Optional(CONF_REMOTE_NODES, default={}): cv.Schema(
                {cv.string: cv.file_}
            ),
            cv.Optional(CONF_REMOTE_LOGS): REMOTE_LOGS_SCHEMA,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_unique_indices,
    validate_od_writer_tpdo,
    lambda config: resolve_remote_rpdo(config),  # noqa: PLW0108 (defined below)
)


def _remote_node(remote_nodes: dict[str, Any], node: str) -> remote.RemoteNode:
    if node not in remote_nodes:
        raise cv.Invalid(f"Unknown node '{node}', add it to '{CONF_REMOTE_NODES}'")
    try:
        return remote.get_remote_node(
            node, CORE.relative_config_path(remote_nodes[node])
        )
    except remote.RemoteError as err:
        raise cv.Invalid(str(err)) from err


def resolve_remote_rpdo(config: ConfigType) -> ConfigType:
    """Resolve the `rpdo:` entries of the entities to the remote state (RemoteNode.state() + node, tpdo_size).

    Raw entries (`node_id`) get a `remote` without `node`: one byte, `tpdo_size` only if given.
    """
    for entity_pos, entity in enumerate(config[CONF_ENTITIES]):
        for rpdo_pos, rpdo in enumerate(entity.get(CONF_RPDO, [])):
            path = [CONF_ENTITIES, entity_pos, CONF_RPDO, rpdo_pos]
            if canopen.CONF_NODE_ID in rpdo:
                entity[CONF_RPDO][rpdo_pos] = {
                    **rpdo,
                    "remote": {
                        "node_id": rpdo[canopen.CONF_NODE_ID],
                        "tpdo": rpdo[CONF_TPDO],
                        "offset": rpdo[CONF_OFFSET],
                        "size": 1,
                        "tpdo_size": rpdo.get(CONF_TPDO_SIZE),
                    },
                }
                continue
            try:
                node = _remote_node(config[CONF_REMOTE_NODES], rpdo[CONF_NODE])
                state = node.state(
                    rpdo.get(CONF_ENTITY), rpdo.get(CONF_INDEX), rpdo.get(CONF_STATE)
                )
            except remote.RemoteError as err:
                raise cv.Invalid(str(err), path=path) from err
            except cv.Invalid as err:
                raise cv.Invalid(str(err), path=path) from err
            entity[CONF_RPDO][rpdo_pos] = {
                **rpdo,
                "remote": {
                    **state,
                    "node": rpdo[CONF_NODE],
                    "tpdo_size": node.tpdo_size(state["tpdo"]),
                },
            }
    return config


def resolve_entity(
    full_config: fv.FinalValidateConfig, entity_config: ConfigType
) -> tuple[str, ConfigType]:
    """(domain, platform config) of an exposed entity, via its declared C++ type."""
    entity_id = entity_config[CONF_ID]
    for declared_id, path in full_config.declare_ids:  # noqa: B007 (path: after the loop)
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


def node_states(full_config: fv.FinalValidateConfig, config: ConfigType) -> list[Any]:
    try:
        return layout.node_states(
            config, lambda entity: resolve_entity(full_config, entity)
        )
    except layout.LayoutError as err:
        path = [CONF_ENTITIES, err.entity_pos] if err.entity_pos is not None else []
        raise cv.Invalid(str(err), path=path) from err


def hub_config(full_config: fv.FinalValidateConfig, canopen_id: Any) -> ConfigType:
    path = full_config.get_path_for_id(canopen_id)[:-1]
    return full_config.get_config_for_path(path)


def _final_validate(config: ConfigType) -> ConfigType:
    full_config = fv.full_config.get()
    try:
        layout.tpdo_mapping(node_states(full_config, config))
    except layout.LayoutError as err:
        raise cv.Invalid(str(err)) from err
    validate_rpdo_sizes(full_config, config)
    validate_remote_entities(full_config, config)
    try:
        remote.rpdo_plan(
            rpdo_sources(config, resolve_remote_slots(full_config, config))
        )
    except remote.PlanError as err:
        raise cv.Invalid(str(err)) from err
    validate_remote_logs(full_config, config)
    return config


def validate_remote_logs(
    full_config: fv.FinalValidateConfig, config: ConfigType
) -> None:
    if (remote_logs := config.get(CONF_REMOTE_LOGS)) is None:
        return
    if "logger" not in full_config:
        raise cv.Invalid(
            f"'{CONF_REMOTE_LOGS}' requires the 'logger' component",
            path=[CONF_REMOTE_LOGS],
        )
    level = remote_logs[CONF_LEVEL]
    logger_level = full_config["logger"][CONF_LEVEL]
    if LOG_LEVEL_SEVERITY.index(level) > LOG_LEVEL_SEVERITY.index(logger_level):
        _LOGGER.warning(
            "%s: level %s is more verbose than the logger level %s, those messages are compiled out",
            CONF_REMOTE_LOGS,
            level,
            logger_level,
        )
    if remote_logs[CONF_TIMEOUT] == LOG_TIMEOUT_NEVER and level == "NONE":
        _LOGGER.warning(
            "%s: 'timeout: never' with 'level: NONE' never expires a host subscription",
            CONF_REMOTE_LOGS,
        )


def validate_rpdo_sizes(
    full_config: fv.FinalValidateConfig, config: ConfigType
) -> None:
    """A remote state must have the size of the local command it's written to."""
    for entity_pos, entity in enumerate(config[CONF_ENTITIES]):
        if not entity.get(CONF_RPDO):
            continue
        domain, platform_config = resolve_entity(full_config, entity)
        caps = layout.entity_caps(domain, platform_config, entity.get(CONF_CAPS))
        local = layout.entity_states(domain, caps, entity.get(CONF_SIZE))
        for rpdo_pos, rpdo in enumerate(entity[CONF_RPDO]):
            path = [CONF_ENTITIES, entity_pos, CONF_RPDO, rpdo_pos]
            src = rpdo["remote"]
            if rpdo[CONF_CMD] >= len(local):
                raise cv.Invalid(
                    f"'{entity[CONF_ID]}' has no command {rpdo[CONF_CMD]}", path=path
                )
            name, size = local[rpdo[CONF_CMD]]
            if size != src["size"]:
                raise cv.Invalid(
                    f"{rpdo_label(src)} is {src['size']} B, but "
                    f"'{entity[CONF_ID]}' command {rpdo[CONF_CMD]} ({name}) is {size} B",
                    path=path,
                )


def validate_remote_entities(
    full_config: fv.FinalValidateConfig, config: ConfigType
) -> None:
    """`platform: canopen_entities` entities may be exposed (for debugging), but get their state from a slot."""
    remote_ids = {
        str(declared_id.id)
        for declared_id, _ in full_config.declare_ids
        if declared_id.type is not None
        and any(declared_id.type.inherits_from(cls) for cls in REMOTE_ENTITY_CLASSES)
    }
    for entity_pos, entity in enumerate(config[CONF_ENTITIES]):
        if str(entity[CONF_ID]) in remote_ids and entity.get(CONF_RPDO):
            raise cv.Invalid(
                f"'{entity[CONF_ID]}' shows a remote state: it can't have '{CONF_RPDO}'",
                path=[CONF_ENTITIES, entity_pos, CONF_RPDO],
            )


def _walk_remote_states(value: Any) -> Iterable[tuple[str, str, str]]:
    """(node, entity, state) of all validated configs marked with CONF_REMOTE_STATE."""
    if isinstance(value, dict):
        if CONF_REMOTE_STATE in value:
            yield tuple(value[CONF_REMOTE_STATE])
        for item in value.values():
            yield from _walk_remote_states(item)
    elif isinstance(value, list):
        for item in value:
            yield from _walk_remote_states(item)


def remote_state_slots(
    full_config: fv.FinalValidateConfig,
) -> list[tuple[str, str, str]]:
    """Sorted, unique (node, entity, state) remote states mirrored in slots (one mapper per node)."""
    return sorted(set(_walk_remote_states(dict(full_config))))


def remote_state_slot(full_config: fv.FinalValidateConfig, key: list[str]) -> int:
    return remote_state_slots(full_config).index(tuple(key))


def resolve_remote_slots(
    full_config: fv.FinalValidateConfig, config: ConfigType
) -> list[tuple[int, dict]]:
    """[(slot n, RemoteNode.state() + node, tpdo_size)] of the mapper."""
    slots = []
    for n, (node_name, entity, state) in enumerate(remote_state_slots(full_config)):
        node = _remote_node(config[CONF_REMOTE_NODES], node_name)
        try:
            info = node.state(entity, None, state)
        except remote.RemoteError as err:
            raise cv.Invalid(str(err)) from err
        slots.append(
            (n, {**info, "node": node_name, "tpdo_size": node.tpdo_size(info["tpdo"])})
        )
    return slots


def rpdo_label(src: dict) -> str:
    """A resolved `rpdo:` entry (its `remote`) in errors."""
    if "node" in src:
        return f"{src['node']} {src['entity']}.{src['state']}"
    return f"node 0x{src['node_id']:02x} TPDO {src['tpdo']} offset {src['offset']}"


def rpdo_sources(config: ConfigType, slots: list[tuple[int, dict]]) -> list[dict]:
    """remote.rpdo_plan() sources: the entities' `rpdo:` entries and the slots."""
    sources = [
        {
            **rpdo["remote"],
            "entity_index": entity[CONF_INDEX],
            CONF_CMD: rpdo[CONF_CMD],
            "label": f"{rpdo_label(rpdo['remote'])} (rpdo of '{entity[CONF_ID]}')",
        }
        for entity in config[CONF_ENTITIES]
        for rpdo in entity.get(CONF_RPDO, ())
    ]
    sources += [
        {
            **state,
            "slot": n,
            "label": f"{state['node']} {state['entity']}.{state['state']} (remote state slot {n})",
        }
        for n, state in slots
    ]
    return sources


FINAL_VALIDATE_SCHEMA = _final_validate


def pdo_mappings(entries: Iterable[int]) -> MockObj:
    """Braced canopen::PdoMapping list (std::initializer_list) of layout.tpdo_mapping() words."""
    items = ", ".join(
        f"{{0x{word >> 16:04X}, {(word >> 8) & 0xFF}, {word & 0xFF}}}"
        for word in entries
    )
    return cg.RawExpression(f"{{{items}}}")


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[canopen.CONF_CANOPEN_ID])
    node_id = hub_config(CORE.config, config[canopen.CONF_CANOPEN_ID])[
        canopen.CONF_NODE_ID
    ]
    canopen.enable_pdo()

    # the ESPHome identity can2mqtt checks, and the version string its firmware scanner finds in the binary
    cg.add(
        hub.set_identity(ESPHOME_VENDOR_ID, ESPHOME_PRODUCT_CODE, ESPHOME_REVISION, 0)
    )
    ts = dt.datetime.now(dt.UTC).strftime("%Y%m%d.%H%M%S")
    cg.add_global(
        cg.RawStatement(
            f'const char *const version_str_{node_id} = "node_id: {node_id:02x}, ver: " ESPHOME_VERSION ".{ts}";'
        )
    )
    cg.add(hub.set_software_version(cg.RawExpression(f"version_str_{node_id} + 18")))

    # the mapper keeps its entities in a StaticVector of this size
    cg.add_define("CANOPEN_ENTITIES_ENTITIES", max(1, len(config[CONF_ENTITIES])))
    var = cg.new_Pvariable(config[CONF_ID], hub)
    await cg.register_component(var, config)
    cg.add(var.set_node_id(node_id))
    if config[CONF_OD_WRITER]:
        cg.add(var.enable_od_writer())

    # TPDOs: all 8 exist (can2mqtt reads them), unused ones disabled
    states = node_states(CORE.config, config)
    mapping = layout.tpdo_mapping(states)
    for tpdo, entries in enumerate(mapping):
        cg.add(
            hub.configure_tpdo(
                tpdo, layout.tpdo_cob_id(tpdo, node_id), pdo_mappings(entries)
            )
        )
    tpdo_layout_hash = layout.layout_hash(mapping)
    cg.add(var.set_layout_hash(tpdo_layout_hash))
    _LOGGER.info(
        "CANopen node 0x%02x TPDO layout (hash %08x):%s",
        node_id,
        tpdo_layout_hash,
        "".join(f"\n  {line}" for line in layout.layout_table(states, node_id))
        or " no TPDOs",
    )

    for entity_config in sorted(config[CONF_ENTITIES], key=lambda e: e[CONF_INDEX]):
        entity = await cg.get_variable(entity_config[CONF_ID])
        domain, platform_config = resolve_entity(CORE.config, entity_config)
        index = entity_config[CONF_INDEX]
        if domain in (layout.DOMAIN_SENSOR, layout.DOMAIN_NUMBER):
            size = entity_config.get(CONF_SIZE, 4)
            # 255 / 65535 are reserved for NaN
            min_value = entity_config.get(CONF_MIN_VALUE, 0.0)
            max_value = entity_config.get(
                CONF_MAX_VALUE, 254.0 if size == 1 else 65534.0
            )
            add = var.add_sensor if domain == layout.DOMAIN_SENSOR else var.add_number
            cg.add(add(entity, index, size, min_value, max_value))
        elif domain == layout.DOMAIN_BINARY_SENSOR:
            cg.add(var.add_binary_sensor(entity, index))
        elif domain == layout.DOMAIN_SWITCH:
            cg.add(var.add_switch(entity, index))
        elif domain in (layout.DOMAIN_LIGHT, layout.DOMAIN_COVER):
            # the OD layout depends on the caps: computed here (layout.py), not from the traits at runtime
            caps = layout.entity_caps(
                domain, platform_config, entity_config.get(CONF_CAPS)
            )
            add = var.add_light if domain == layout.DOMAIN_LIGHT else var.add_cover
            cg.add(add(entity, index, caps))
        elif domain == layout.DOMAIN_ALARM:
            cg.add(var.add_alarm(entity, index))

    # remote state slots and the RPDOs from the remote nodes' TPDOs (slots and entity `rpdo:`)
    slots = resolve_remote_slots(CORE.config, config)
    if slots:
        cg.add_define("CANOPEN_ENTITIES_REMOTE_SLOTS", len(slots))
    for n, state in slots:
        _LOGGER.info(
            "CANopen node 0x%02x remote state slot %d (0x%04x): %s %s.%s",
            node_id,
            n,
            remote.REMOTE_STATE_INDEX + n,
            state["node"],
            state["entity"],
            state["state"],
        )
        cg.add(var.add_remote_state(n, state["size"]))
    for n, (remote_node_id, tpdo, entries) in enumerate(
        remote.rpdo_plan(rpdo_sources(config, slots))
    ):
        cg.add(
            hub.configure_rpdo(
                n,
                layout.tpdo_cob_id(tpdo, remote_node_id),
                pdo_mappings(remote.co_link_words(entries)),
            )
        )

    if (remote_logs := config.get(CONF_REMOTE_LOGS)) is not None:
        await remote_logs_to_code(remote_logs, hub, config[canopen.CONF_CANOPEN_ID])


def enable_sdo_block(canopen_id: Any) -> None:
    """SDO block transfer, with the hub's sdo_block_size or DEFAULT_SDO_BLOCK_SIZE."""
    if canopen.CONF_SDO_BLOCK_SIZE not in hub_config(CORE.config, canopen_id):
        canopen.enable_sdo_block(DEFAULT_SDO_BLOCK_SIZE)


async def remote_logs_to_code(
    config: ConfigType, hub: MockObj, canopen_id: Any
) -> None:
    # Log Data uploads are block uploads; their block size is the client's (0x3100:7), not the hub's download cap
    enable_sdo_block(canopen_id)
    request_log_listener()
    cg.add_define("USE_CANOPEN_ENTITIES_LOGS")
    cg.add_define("CANOPEN_ENTITIES_LOG_BUFFER_SIZE", config[CONF_BUFFER_SIZE])
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_default_level(LOG_LEVEL_SEVERITY.index(config[CONF_LEVEL])))
    timeout = config[CONF_TIMEOUT]
    if timeout != LOG_TIMEOUT_NEVER:
        cg.add(var.set_timeout(timeout.total_milliseconds))
    cg.add(var.set_block_size(config[CONF_BLOCK_SIZE]))
    cg.add(var.set_canopen(hub))
    # now, not in setup(): codegen runs before App.setup(), so the setup logs of all components are buffered
    cg.add(var.start())


# ---- links to other nodes: actions and `platform: canopen_entities` entities ----


def _raw_remote_node(node: str) -> remote.RemoteNode:
    """A remote node of the local raw config (packages and substitutions applied, not validated yet), available
    while actions and platforms are validated."""
    mapper = (CORE.raw_config or {}).get("canopen_entities") or {}
    if isinstance(mapper, list):
        mapper = mapper[0] if mapper else {}
    return _remote_node(mapper.get(CONF_REMOTE_NODES) or {}, node)


# RemoteCommandAction write encodings, see RemoteCmdEncoding in canopen_entities.h
ENC_RAW = 0
ENC_PERCENTAGE = 1
ENC_COLOR_TEMP = 2
ENC_SCALED = 3
ENC_REAL32 = 4

# value of a toggle write: C++ expression over the raw remote state `s` (slot value)
TOGGLE_BOOL = ("toggle", "s ? 0.0f : 1.0f")
# moving (1 opening, 3 closing) -> stop (0); closed (2) -> open (1); open (0) -> close (2)
TOGGLE_COVER = ("toggle", "(s == 1 || s == 3) ? 0.0f : (s == 2 ? 1.0f : 2.0f)")
# writes per action: light turn_on writes brightness, color temperature and state
MAX_WRITES = 3

REMOTE_ACTION_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(CANopenEntities),
        cv.Required(CONF_NODE): cv.string,
        cv.Exclusive(CONF_ENTITY, "remote_entity"): cv.string,
        cv.Exclusive(CONF_INDEX, "remote_entity"): cv.int_,
    }
)


def _write(
    cmd: int,
    value: Any,
    size: int = 1,
    encoding: int = ENC_RAW,
    min_value: float = 0.0,
    max_value: float = 0.0,
) -> dict[str, Any]:
    return {
        CONF_CMD: cmd,
        CONF_SIZE: size,
        "encoding": encoding,
        CONF_MIN_VALUE: min_value,
        CONF_MAX_VALUE: max_value,
        "value": value,
    }


class _Target:
    """Remote entity an action is validated against."""

    def __init__(self, action: str, config: ConfigType) -> None:
        self.action = action
        self.node = _raw_remote_node(config[CONF_NODE])
        try:
            self.entity = self.node.entity(
                config.get(CONF_ENTITY), config.get(CONF_INDEX)
            )
        except remote.RemoteError as err:
            raise cv.Invalid(str(err)) from err
        self.domain = self.node.domain(index=self.entity["index"])
        self.states = self.node.state_names(index=self.entity["index"])

    def cmd(self, state: str) -> int:
        """Command index of a remote state (states and commands are in the same order)."""
        if state not in self.states:
            raise cv.Invalid(
                f"{self.action}: {self.node.name} {self.domain} '{self.entity['id']}' has no "
                f"{state} (states: {', '.join(self.states)})"
            )
        return self.states.index(state)

    def check_state(self, state: str) -> None:
        """The remote state must be sent in a TPDO to be read (toggles)."""
        try:
            self.node.state(index=self.entity["index"], state=state)
        except remote.RemoteError as err:
            raise cv.Invalid(f"{self.action}: {err}") from err


def _remote_action(
    name: str, domains: tuple[str, ...], writes, schema=None, toggle: bool = False
) -> None:
    """Register `canopen_entities.<domain>.<action>`, sending the commands `writes(config, target)` returns."""

    def validate(config: ConfigType) -> ConfigType:
        if CONF_ENTITY not in config and CONF_INDEX not in config:
            raise cv.Invalid("'entity' or 'index' is required")
        target = _Target(name, config)
        if target.domain not in domains:
            raise cv.Invalid(
                f"{name}: {target.node.name} '{target.entity['id']}' is a {target.domain}, "
                f"expected {' or '.join(domains)}"
            )
        if not target.node.od_writer:
            raise cv.Invalid(
                f"{name}: {target.node.name} has the OD writer off ({target.node.path})"
            )
        config = {
            **config,
            CONF_ENTITY: str(target.entity["id"]),
            "node_id": target.node.node_id,
            CONF_INDEX: target.entity["index"],
            "_writes": writes(config, target),
        }
        if toggle:
            target.check_state(CONF_STATE)
            config[CONF_REMOTE_STATE] = [
                config[CONF_NODE],
                config[CONF_ENTITY],
                CONF_STATE,
            ]
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
        cg.add(var.set_target(config["node_id"], config[CONF_INDEX]))
        for write in config["_writes"]:
            value = write["value"]
            if isinstance(value, tuple) and value[0] == "toggle":
                n = remote_state_slot(CORE.config, config[CONF_REMOTE_STATE])
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
                    write[CONF_CMD],
                    write[CONF_SIZE],
                    write["encoding"],
                    write[CONF_MIN_VALUE],
                    write[CONF_MAX_VALUE],
                    value,
                )
            )
        return var


def _const_writes(*values):
    return lambda config, target: [_write(0, value) for value in values]


def _light_turn_on_writes(config: ConfigType, target: _Target) -> list[dict]:
    writes = []
    if "brightness" in config:
        writes.append(
            _write(
                target.cmd("brightness"), config["brightness"], encoding=ENC_PERCENTAGE
            )
        )
    if "color_temperature" in config:
        writes.append(
            _write(
                target.cmd("color_temp"),
                config["color_temperature"],
                encoding=ENC_COLOR_TEMP,
            )
        )
    # brightness alone doesn't switch an ESPHome light on
    return writes + [_write(0, 1)]


def _cover_control_writes(config: ConfigType, target: _Target) -> list[dict]:
    writes = []
    if config.get("stop"):
        writes.append(_write(0, 0))
    writes.extend(
        _write(target.cmd(option), config[option], encoding=ENC_PERCENTAGE)
        for option in ("position", "tilt")
        if option in config
    )
    if not writes:
        raise cv.Invalid(
            "canopen_entities.cover.control: set 'stop', 'position' or 'tilt'"
        )
    return writes


def _number_set_writes(config: ConfigType, target: _Target) -> list[dict]:
    size = target.entity.get("size", 4)
    if size == 4:
        return [_write(0, config["value"], size=4, encoding=ENC_REAL32)]
    min_value, max_value = target.node.scaling(target.entity)
    return [_write(0, config["value"], size, ENC_SCALED, min_value, max_value)]


_LIGHT_SWITCH = (layout.DOMAIN_LIGHT, layout.DOMAIN_SWITCH)
_SWITCH_LIGHT = (layout.DOMAIN_SWITCH, layout.DOMAIN_LIGHT)
_COVER = (layout.DOMAIN_COVER,)

_remote_action(
    "canopen_entities.light.turn_on",
    _LIGHT_SWITCH,
    _light_turn_on_writes,
    {
        cv.Optional("brightness"): cv.templatable(cv.percentage),
        cv.Optional("color_temperature"): cv.templatable(cv.color_temperature),
    },
)
_remote_action("canopen_entities.light.turn_off", _LIGHT_SWITCH, _const_writes(0))
_remote_action(
    "canopen_entities.light.toggle",
    _LIGHT_SWITCH,
    _const_writes(TOGGLE_BOOL),
    toggle=True,
)
_remote_action("canopen_entities.switch.turn_on", _SWITCH_LIGHT, _const_writes(1))
_remote_action("canopen_entities.switch.turn_off", _SWITCH_LIGHT, _const_writes(0))
_remote_action(
    "canopen_entities.switch.toggle",
    _SWITCH_LIGHT,
    _const_writes(TOGGLE_BOOL),
    toggle=True,
)
_remote_action("canopen_entities.cover.open", _COVER, _const_writes(1))
_remote_action("canopen_entities.cover.close", _COVER, _const_writes(2))
_remote_action("canopen_entities.cover.stop", _COVER, _const_writes(0))
_remote_action(
    "canopen_entities.cover.toggle", _COVER, _const_writes(TOGGLE_COVER), toggle=True
)
_remote_action(
    "canopen_entities.cover.control",
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
        f"canopen_entities.alarm_control_panel.{_action}",
        (layout.DOMAIN_ALARM,),
        _const_writes(_code),
    )
_remote_action(
    "canopen_entities.number.set",
    (layout.DOMAIN_NUMBER,),
    _number_set_writes,
    {cv.Required("value"): cv.templatable(cv.float_)},
)


# `platform: canopen_entities` binary_sensor / sensor / text_sensor (binary_sensor.py, ...)
RemoteBinarySensor = ns.class_(
    "RemoteBinarySensor",
    cg.esphome_ns.namespace("binary_sensor").class_("BinarySensor"),
)
RemoteSensor = ns.class_(
    "RemoteSensor", cg.esphome_ns.namespace("sensor").class_("Sensor")
)
RemoteTextSensor = ns.class_(
    "RemoteTextSensor", cg.esphome_ns.namespace("text_sensor").class_("TextSensor")
)
REMOTE_ENTITY_CLASSES = (RemoteBinarySensor, RemoteSensor, RemoteTextSensor)

REMOTE_STATE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MAPPER_ID): cv.use_id(CANopenEntities),
        cv.Required(CONF_NODE): cv.string,
        cv.Required(CONF_ENTITY): cv.string,
        cv.Optional(CONF_STATE): cv.string,
    }
)


def validate_remote_state(platform: str, kinds: list[str]):
    """Resolve the remote state of a `platform: canopen_entities` entity and check its kind."""

    def validator(config: ConfigType) -> ConfigType:
        node = _raw_remote_node(config[CONF_NODE])
        try:
            state = node.state(config[CONF_ENTITY], None, config.get(CONF_STATE))
        except remote.RemoteError as err:
            raise cv.Invalid(str(err)) from err
        if state["kind"] not in kinds:
            raise cv.Invalid(
                f"{platform} can't show {config[CONF_NODE]} {state['entity']}.{state['state']} "
                f"({state['kind']}), supported: {', '.join(kinds)}"
            )
        return {
            **config,
            CONF_STATE: state["state"],
            "_remote": state,
            CONF_REMOTE_STATE: [config[CONF_NODE], state["entity"], state["state"]],
        }

    return validator


async def attach_remote_state(var: MockObj, config: ConfigType) -> None:
    """Subscribe a `platform: canopen_entities` entity to its remote state slot."""
    mapper = await cg.get_variable(config[CONF_MAPPER_ID])
    n = remote_state_slot(CORE.config, config[CONF_REMOTE_STATE])
    state = config["_remote"]
    cg.add(
        var.attach(
            mapper,
            n,
            layout.KIND_CODES[state["kind"]],
            state["size"],
            state.get("min_value", 0.0),
            state.get("max_value", 0.0),
        )
    )

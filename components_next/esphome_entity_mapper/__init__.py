"""ESPHome entities in a CANopen object dictionary, for can2mqtt (OBJECT_DICTIONARY.md).

Sits on top of the `canopen` hub (`canopen_id`): entity metadata and states at 0x2000 + 16 * N, TPDOs computed by
layout.py, the layout hash at 0x3002, and the ESPHome identity can2mqtt checks.
"""

import datetime
import logging

import esphome.codegen as cg
from esphome.components import canopen
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.core import CORE
import esphome.final_validate as fv

from . import layout

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@mrk-its"]
DEPENDENCIES = ["canopen"]

CONF_ENTITIES = "entities"
CONF_INDEX = "index"
CONF_SIZE = "size"
CONF_MIN_VALUE = "min_value"
CONF_MAX_VALUE = "max_value"
CONF_TPDO = "tpdo"
CONF_NUMBER = "number"
CONF_IMMEDIATE = "immediate"
CONF_CAPS = "caps"

# what can2mqtt accepts (can2mqtt.py ESPHOME_VENDOR_ID / ESPHOME_PRODUCT_CODE); revision 1: entity types in 0x2001
ESPHOME_VENDOR_ID = 0xA59A08F5
ESPHOME_PRODUCT_CODE = 0x6BDFA1D9
ESPHOME_REVISION = 1

ns = cg.esphome_ns.namespace("esphome_entity_mapper")
EntityMapper = ns.class_("EntityMapper", cg.Component)

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
    }
)


def validate_unique_indices(config):
    seen = set()
    for pos, entity in enumerate(config[CONF_ENTITIES]):
        if entity[CONF_INDEX] in seen:
            raise cv.Invalid(
                f"index {entity[CONF_INDEX]} is used twice",
                path=[CONF_ENTITIES, pos, CONF_INDEX],
            )
        seen.add(entity[CONF_INDEX])
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EntityMapper),
            cv.GenerateID(canopen.CONF_CANOPEN_ID): cv.use_id(
                canopen.CANopenComponent
            ),
            cv.Optional(CONF_ENTITIES, default=[]): cv.ensure_list(ENTITY_SCHEMA),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_unique_indices,
)


def resolve_entity(full_config, entity_config):
    """(domain, platform config) of an exposed entity, via its declared C++ type."""
    entity_id = entity_config[CONF_ID]
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


def node_states(full_config, config):
    try:
        return layout.node_states(
            config, lambda entity: resolve_entity(full_config, entity)
        )
    except layout.LayoutError as err:
        path = [CONF_ENTITIES, err.entity_pos] if err.entity_pos is not None else []
        raise cv.Invalid(str(err), path=path) from err


def hub_config(full_config, canopen_id):
    path = full_config.get_path_for_id(canopen_id)[:-1]
    return full_config.get_config_for_path(path)


def _final_validate(config):
    full_config = fv.full_config.get()
    try:
        layout.tpdo_mapping(node_states(full_config, config))
    except layout.LayoutError as err:
        raise cv.Invalid(str(err)) from err
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


def pdo_mappings(entries):
    """canopen::PdoMapping initializer of layout.tpdo_mapping() words."""
    items = ", ".join(
        f"{{0x{word >> 16:04X}, {(word >> 8) & 0xFF}, {word & 0xFF}}}"
        for word in entries
    )
    return cg.RawExpression(f"std::vector<canopen::PdoMapping>{{{items}}}")


async def to_code(config):
    hub = await cg.get_variable(config[canopen.CONF_CANOPEN_ID])
    node_id = hub_config(CORE.config, config[canopen.CONF_CANOPEN_ID])[
        canopen.CONF_NODE_ID
    ]
    canopen.enable_pdo()

    # the ESPHome identity can2mqtt checks, and the version string its firmware scanner finds in the binary
    cg.add(
        hub.set_identity(
            ESPHOME_VENDOR_ID, ESPHOME_PRODUCT_CODE, ESPHOME_REVISION, 0
        )
    )
    ts = datetime.datetime.now(datetime.UTC).strftime("%Y%m%d.%H%M%S")
    cg.add_global(
        cg.RawStatement(
            f'const char *const version_str_{node_id} = "node_id: {node_id:02x}, ver: " ESPHOME_VERSION ".{ts}";'
        )
    )
    cg.add(
        hub.set_software_version(cg.RawExpression(f"version_str_{node_id} + 18"))
    )

    var = cg.new_Pvariable(config[CONF_ID], hub)
    await cg.register_component(var, config)

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

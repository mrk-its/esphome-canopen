from itertools import groupby
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome import automation
from esphome.const import CONF_ID, CONF_TRIGGER_ID
from esphome.components.canbus import CanbusComponent


ns = cg.esphome_ns.namespace('canopen')
CanopenComponent = ns.class_(
    'CanopenComponent',
    cg.Component,
)

PreOperationalTrigger = ns.class_("PreOperationalTrigger", automation.Trigger.template())
OperationalTrigger = ns.class_("OperationalTrigger", automation.Trigger.template())

CONF_ENTITIES = "entities"

DEPENDENCIES = ["ota"]

CSDO_SCHEMA = cv.Schema({
    cv.Required("node_id"): cv.int_,
    cv.Optional("tx_id", 0x600): cv.int_,
    cv.Optional("rx_id", 0x580): cv.int_,
})

RPDO_SCHEMA = cv.Schema({
    cv.Required("node_id"): cv.int_,
    cv.Required("tpdo"): cv.int_,
    cv.Required("offset"): cv.int_,
    cv.Optional("cmd", 0): cv.int_
})

ENTITY_SCHEMA = cv.Schema({
    cv.Required("id"): cv.use_id(cg.EntityBase),
    cv.Required("index"): cv.int_,
    cv.Optional("size"): cv.int_,
    cv.Optional("min_value", 0): cv.float_,
    cv.Optional("max_value", 0): cv.float_,
    cv.Optional("tpdo"): cv.int_,
    cv.Optional("rpdo"): cv.ensure_list(RPDO_SCHEMA),
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CanopenComponent),
    cv.Required("canbus_id"): cv.use_id(CanbusComponent),
    cv.Required("node_id"): cv.int_,
    # cv.Optional("status"): STATUS_ENTITY_SCHEMA,
    cv.Optional("csdo"): cv.ensure_list(CSDO_SCHEMA),
    cv.Required(CONF_ENTITIES): cv.ensure_list(ENTITY_SCHEMA),
    cv.Optional("on_pre_operational"): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(PreOperationalTrigger),
    }),
    cv.Optional("on_operational"): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OperationalTrigger),
    }),
    cv.Optional("state_update_delay", "50ms"): cv.positive_time_period_microseconds,
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    cg.add_library("canopenstack=https://github.com/mrk-its/canopen-stack#dev", "0.0.0")
    # cg.add_library("canopenstack=file:///home/mrk/canopen-stack", "0.0.0")
    canbus = yield cg.get_variable(config["canbus_id"])
    node_id = config["node_id"]
    canopen = cg.new_Pvariable(config[CONF_ID], canbus, node_id)
    cg.add(canopen.set_state_update_delay(config["state_update_delay"]))
    yield cg.register_component(canopen, config)

    entities = sorted(config.get(CONF_ENTITIES, []), key=lambda x: x['index'])
    assert len(entities) == len(set(e['index'] for e in entities)), "All entity indices must be unique!"
    for entity_config in entities:
        entity = yield cg.get_variable(entity_config["id"])
        tpdo = entity_config.get("tpdo", -1)
        size = entity_config.get("size")
        if size in (1, 2):
            min_val = entity_config.get("min_value", 0)
            max_val = entity_config.get("max_value", 255 if size == 1 else 65535)
            cg.add(canopen.add_entity(entity, entity_config["index"], tpdo, size, min_val, max_val))
        else:
            cg.add(canopen.add_entity(entity, entity_config["index"], tpdo))

    for num, csdo in enumerate(config.get("csdo", ())):
        cg.add(canopen.setup_csdo(num, csdo["node_id"], csdo["tx_id"], csdo["rx_id"]))

    for conf in config.get("on_operational", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        yield automation.build_automation(trigger, [], conf)
        cg.add(canopen.add_trigger(trigger))

    for conf in config.get("on_pre_operational", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        yield automation.build_automation(trigger, [], conf)
        cg.add(canopen.add_trigger(trigger))

    rpdo_entities = [
        {**rpdo, "entity_index": entity["index"]}
        for entity in entities
        for rpdo in entity.get("rpdo", ())
    ]
    rpdo_entities.sort(key=lambda rpdo: (rpdo["node_id"], rpdo["tpdo"], rpdo["offset"]))
    for idx, ((node_id, tpdo), rpdos) in enumerate(
        groupby(rpdo_entities, key=lambda rpdo: (rpdo["node_id"], rpdo["tpdo"]))
    ):
        cg.add(canopen.add_rpdo_node(idx, node_id, tpdo))
        rpdos = list(rpdos)
        curr_offs = 0
        for rpdo in rpdos:
            assert rpdo["offset"] >= curr_offs, f"RPDO: invalid TPDO offset {rpdo}"
            if rpdo["offset"] > curr_offs:
                cg.add(canopen.add_rpdo_dummy(idx, rpdo["offset"] - curr_offs))
                curr_offs = rpdo["offset"]
            cg.add(canopen.add_rpdo_entity_cmd(idx, rpdo["entity_index"], rpdo["cmd"]))
            curr_offs += 1

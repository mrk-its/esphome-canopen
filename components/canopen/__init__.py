from itertools import groupby
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID
from esphome.components.canbus import CanbusComponent


ns = cg.esphome_ns.namespace('canopen')
CanopenComponent = ns.class_(
    'CanopenComponent',
    cg.Component,
)

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
})

def to_code(config):
    cg.add_library("canopenstack=https://github.com/mrk-its/canopen-stack", "0.0.0")
    # cg.add_library("canopenstack=file:///home/mrk/canopen-stack", "0.0.0")
    canopen = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(canopen, config)
    canbus = yield cg.get_variable(config["canbus_id"])
    node_id = config["node_id"]
    cg.add(canopen.initialize(canbus, node_id))
    entities = sorted(config.get(CONF_ENTITIES, []), key=lambda x: x['index'])
    assert len(entities) == len(set(e['index'] for e in entities)), "All entity indices must be unique!"
    for entity_config in entities:
        entity = yield cg.get_variable(entity_config["id"])
        tpdo = entity_config.get("tpdo", -1)
        cg.add(canopen.add_entity(entity, entity_config["index"], tpdo))

    for num, csdo in enumerate(config.get("csdo", ())):
        cg.add(canopen.setup_csdo(num, csdo["node_id"], csdo["tx_id"], csdo["rx_id"]))

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

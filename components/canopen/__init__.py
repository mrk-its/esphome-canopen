from itertools import groupby
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome import automation
from esphome.const import CONF_ID, CONF_TRIGGER_ID
from esphome.components.canbus import CanbusComponent
from esphome.components.mqtt import MQTTClientComponent
from esphome.core import coroutine_with_priority

# from .ota import CanopenOTAComponent

ns = cg.esphome_ns.namespace('canopen')
CanopenComponent = ns.class_(
    'CanopenComponent',
    cg.Component,
)

PreOperationalTrigger = ns.class_("PreOperationalTrigger", automation.Trigger.template())
OperationalTrigger = ns.class_("OperationalTrigger", automation.Trigger.template())
HbConsumerEventTrigger = ns.class_("HbConsumerEventTrigger", automation.Trigger.template())
CmdTriggerUInt8 = ns.class_("CmdTriggerUInt8", automation.Trigger.template())
CmdTriggerUInt16 = ns.class_("CmdTriggerUInt16", automation.Trigger.template())
CmdTriggerUInt32 = ns.class_("CmdTriggerUInt32", automation.Trigger.template())
CmdTriggerInt8 = ns.class_("CmdTriggerInt8", automation.Trigger.template())
CmdTriggerInt16 = ns.class_("CmdTriggerInt16", automation.Trigger.template())
CmdTriggerInt32 = ns.class_("CmdTriggerInt32", automation.Trigger.template())

CONF_ENTITIES = "entities"

DEPENDENCIES = []

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
    cv.Optional("min_value"): cv.float_,
    cv.Optional("max_value"): cv.float_,
    cv.Optional("tpdo"): cv.int_,
    cv.Optional("rpdo"): cv.ensure_list(RPDO_SCHEMA),
})

def template_entity_cmd_schema(type, trigger):
    return cv.Schema({
        cv.Required("type"): type,
        cv.Required("handler"):
            automation.validate_automation({
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(trigger),
            }),
    })

TEMPLATE_ENTITY_METADATA_SCHEMA = cv.Schema({
    cv.Required("type"): cv.int_,
    cv.Optional("name", ""): cv.string,
    cv.Optional("device_class", ""): cv.string,
    cv.Optional("unit", ""): cv.string,
    cv.Optional("state_class", ""): cv.string,
})

TEMPLATE_ENTITY_STATE_SCHEMA = cv.Schema({
    cv.Required("type"): cv.string,
})

TEMPLATE_ENTITY_CMD_SCHEMA = cv.vol.Union(
    template_entity_cmd_schema("uint8", CmdTriggerUInt8),
    template_entity_cmd_schema("uint16", CmdTriggerUInt16),
    template_entity_cmd_schema("uint32", CmdTriggerUInt32),
    discriminant=lambda val, alt: filter(lambda x: x.schema['type'] == val['type'], alt)
)

TEMPLATE_ENTITY = cv.Schema({
    cv.Required("index"): cv.int_,
    cv.Optional("tpdo", -1): cv.int_,
    cv.Optional("commands"): cv.ensure_list(TEMPLATE_ENTITY_CMD_SCHEMA),
    cv.Optional("states"): cv.ensure_list(TEMPLATE_ENTITY_STATE_SCHEMA),
    cv.Optional("metadata"): TEMPLATE_ENTITY_METADATA_SCHEMA,
})

HB_CLIENT_SCHEMA = cv.Schema({
    cv.Required("node_id"): cv.int_,
    cv.Required("timeout"): cv.positive_time_period_milliseconds,
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CanopenComponent),
    cv.Optional("canbus_id"): cv.use_id(CanbusComponent),
    # cv.GenerateID("ota_id"): cv.use_id(CanopenOTAComponent),
    cv.Optional("mqtt_id"): cv.use_id(MQTTClientComponent),
    cv.Required("node_id"): cv.int_,
    # cv.Optional("status"): STATUS_ENTITY_SCHEMA,
    cv.Optional("csdo"): cv.ensure_list(CSDO_SCHEMA),
    cv.Required(CONF_ENTITIES): cv.ensure_list(ENTITY_SCHEMA),
    cv.Optional("template_entities"): cv.ensure_list(TEMPLATE_ENTITY),
    cv.Optional("on_pre_operational"): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(PreOperationalTrigger),
    }),
    cv.Optional("on_operational"): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(OperationalTrigger),
    }),
    cv.Optional("on_hb_consumer_event"): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(HbConsumerEventTrigger),
    }),
    cv.Optional("state_update_delay", "100ms"): cv.positive_time_period_microseconds,
    cv.Optional("heartbeat_interval", "5000ms"): cv.positive_time_period_milliseconds,
    cv.Optional("heartbeat_clients"): cv.ensure_list(HB_CLIENT_SCHEMA),
    cv.Optional("sw_version"): cv.string,
    cv.Optional("hw_version"): cv.string
}).extend(cv.COMPONENT_SCHEMA)

TYPE_TO_CANOPEN_TYPE = {
    "uint8": (cg.RawExpression("CO_TUNSIGNED8"), 1),
    "uint16": (cg.RawExpression("CO_TUNSIGNED16"), 2),
    "uint32": (cg.RawExpression("CO_TUNSIGNED32"), 4),
    "int8": (cg.RawExpression("CO_TSIGNED8"), 1),
    "int16": (cg.RawExpression("CO_TSIGNED16"), 2),
    "int32": (cg.RawExpression("CO_TSIGNED32"), 4),
}

# @coroutine_with_priority(100.0)
# async def to_code(config):
#     cg.add_define("USE_CANBUS")


def to_code(config):
    cg.add_library("canopenstack=https://github.com/mrk-its/canopen-stack#dev", None)
    cg.add_library("micro_miniz=https://github.com/rzeldent/micro-miniz#main", None)
    # cg.add_library("canopenstack=file:///home/mrk/canopen-stack", "0.0.0")
    node_id = config["node_id"]
    canopen = cg.new_Pvariable(config[CONF_ID], node_id)

    if "canbus_id" in config:
        cg.add_define("USE_CANBUS")
        canbus = yield cg.get_variable(config["canbus_id"])
        cg.add(canopen.set_canbus(canbus))

    # if "ota_id" in config:
    #     ota = yield cg.get_variable(config["ota_id"])
    #     cg.add(canopen.set_ota(ota))

    if "mqtt_id" in config:
        mqtt_client = yield cg.get_variable(config["mqtt_id"])
        cg.add(canopen.set_mqtt_client(mqtt_client))

    cg.add(canopen.set_state_update_delay(config["state_update_delay"]))
    cg.add(canopen.set_heartbeat_interval(config["heartbeat_interval"]))
    hw_version = config.get("hw_version")
    sw_version = config.get("sw_version")
    if hw_version:
        cg.add(canopen.od_set_string(0x1009, 0, hw_version))
    if sw_version:
        cg.add(canopen.od_set_string(0x100a, 0, sw_version))

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

    for tmpl_entity in config.get("template_entities", []):
        index = tmpl_entity["index"]
        metadata = tmpl_entity.get("metadata")
        if metadata:
            cg.add(canopen.od_add_metadata(index, metadata["type"], metadata["name"], metadata["device_class"], metadata["unit"], metadata["state_class"]))
        for state in tmpl_entity.get("states", ()):
            _type, size = TYPE_TO_CANOPEN_TYPE[state["type"]]
            cg.add(canopen.od_add_state(index, _type, 0, size, tmpl_entity["tpdo"]))
            pass
        for cmd in tmpl_entity.get("commands", ()):
            for handler in cmd["handler"]:
                trigger = cg.new_Pvariable(handler[CONF_TRIGGER_ID])
                yield automation.build_automation(trigger, [(getattr(cg, cmd["type"]), "x"),], handler)
                cg.add(canopen.add_entity_cmd(index, tmpl_entity.get("tpdo", -1), trigger))

    for num, csdo in enumerate(config.get("csdo", ())):
        cg.add(canopen.setup_csdo(num, csdo["node_id"], csdo["tx_id"], csdo["rx_id"]))
    for num, client in enumerate(config.get("heartbeat_clients", ()), 1):
        cg.add(canopen.setup_heartbeat_client(num, client["node_id"], client["timeout"]))

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
        yield automation.build_automation(trigger, [(cg.uint8, "node_id"),], conf)
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

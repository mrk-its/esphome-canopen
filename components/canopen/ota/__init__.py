import logging

import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components.ota import BASE_OTA_SCHEMA, ota_to_code, OTAComponent
from esphome.config_helpers import merge_config
from esphome.const import (
    CONF_ID,
)
from esphome.core import coroutine_with_priority
from esphome import automation
from .. import CanopenComponent

_LOGGER = logging.getLogger(__name__)


CODEOWNERS = ["mrk@sed.pl"]
AUTO_LOAD = ["md5"]
DEPENDENCIES = ["canopen"]

ns = cg.esphome_ns.namespace("canopen")

CanopenOTAComponent = ns.class_("CanopenOTAComponent", OTAComponent)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CanopenOTAComponent),
            cv.GenerateID("canopen_id"): cv.use_id(CanopenComponent),
        }
    )
    .extend(BASE_OTA_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


@coroutine_with_priority(52.0)
async def to_code(config):
    cg.add_define("USE_CANOPEN_OTA")
    var = cg.new_Pvariable(config[CONF_ID])
    await ota_to_code(var, config)
    await cg.register_component(var, config)

    canopen = await cg.get_variable(config["canopen_id"])
    cg.add(canopen.set_ota(var))

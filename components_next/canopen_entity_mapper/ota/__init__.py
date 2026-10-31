"""`ota: - platform: canopen_entity_mapper`: CAN OTA through OD 0x3000 (can2mqtt's firmware upload).

Needs SDO block transfer in the hub: the hub's `sdo_block_size`, or DEFAULT_SDO_BLOCK_SIZE when it isn't set.
"""

import esphome.codegen as cg
from esphome.components import canopen
from esphome.components.ota import BASE_OTA_SCHEMA, OTAComponent, ota_to_code
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.core import CORE
from esphome.types import ConfigType

from .. import enable_sdo_block, ns

CODEOWNERS = ["@mrk-its"]
DEPENDENCIES = ["canopen"]

CONF_REBOOT = "reboot"

# MCUboot swap method (platform: zephyr of the esphomeplatformzephyr fork only)
try:
    from esphome.components.ota import SWAP_METHOD_SCHEMA
except ImportError:
    SWAP_METHOD_SCHEMA = {}

CanopenOTAComponent = ns.class_("CanopenOTAComponent", OTAComponent)


def _validate_swap_method(config: ConfigType) -> ConfigType:
    if not SWAP_METHOD_SCHEMA:
        return config
    # deferred: esphome.components.zephyr is heavy
    from esphome.components.zephyr.mcuboot import validate_swap_method

    return validate_swap_method(config)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CanopenOTAComponent),
            cv.GenerateID(canopen.CONF_CANOPEN_ID): cv.use_id(canopen.CANopenComponent),
            # reboot into the new image after the update
            cv.Optional(CONF_REBOOT, default=True): cv.boolean,
            **SWAP_METHOD_SCHEMA,
        }
    )
    .extend(BASE_OTA_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    _validate_swap_method,
)


def _compression() -> bool:
    """zlib (miniz) inflate needs ~43 KB of heap: not on stm32 / zephyr."""
    return not (getattr(CORE, "is_zephyr", False) or getattr(CORE, "is_stm32", False))


def FILTER_SOURCE_FILES() -> list[str]:
    """miniz only with compression (it doesn't build on zephyr: no utime())."""
    return [] if _compression() else ["miniz.c"]


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await ota_to_code(var, config)
    await cg.register_component(var, config)

    hub = await cg.get_variable(config[canopen.CONF_CANOPEN_ID])
    enable_sdo_block(config[canopen.CONF_CANOPEN_ID])
    if _compression():
        cg.add_define("OTA_COMPRESSION")
    cg.add(var.set_canopen(hub))
    if not config[CONF_REBOOT]:
        cg.add(var.set_reboot(False))

    if SWAP_METHOD_SCHEMA and getattr(CORE, "is_zephyr", False):
        from esphome.components.zephyr.mcuboot import apply_swap_method

        apply_swap_method(config)

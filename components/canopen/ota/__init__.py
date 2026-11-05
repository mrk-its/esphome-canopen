"""`ota: - platform: canopen`: firmware download over CAN on the program download objects of CiA 302-3.

0x1F50 program data, 0x1F51 program control, 0x1F56 program identification (CRC32), 0x1F57 flash status. Only built
with this platform: nothing here is compiled for a node without it.
"""

import logging

import esphome.codegen as cg
from esphome.components import canopen
from esphome.components.ota import BASE_OTA_SCHEMA, OTAComponent, ota_to_code
import esphome.config_validation as cv
from esphome.const import CONF_DISABLED, CONF_ID
from esphome.core import CORE
import esphome.final_validate as fv
from esphome.types import ConfigType

CODEOWNERS = ["@mrk-its"]
DEPENDENCIES = ["canopen"]

_LOGGER = logging.getLogger(__name__)

CONF_REBOOT = "reboot"
# segments per SDO block when the hub's sdo_block_size isn't set: must fit the canbus platform's RX queue
DEFAULT_SDO_BLOCK_SIZE = 16

# MCUboot swap method (Zephyr builds of the esphomeplatformzephyr fork only)
try:
    from esphome.components.ota import SWAP_METHOD_SCHEMA
except ImportError:
    SWAP_METHOD_SCHEMA = {}

CANopenOTAComponent = canopen.ns.class_("CANopenOTAComponent", OTAComponent)


def _is_zephyr() -> bool:
    return getattr(CORE, "using_zephyr_rtos", False) or getattr(
        CORE, "is_zephyr", False
    )


def _mcuboot():
    """The fork's swap method helpers. Deferred: esphome.components.zephyr is heavy."""
    try:
        from esphome.components.zephyr import apply_swap_method, validate_swap_method
    except ImportError:
        from esphome.components.zephyr.mcuboot import (
            apply_swap_method,
            validate_swap_method,
        )
    return validate_swap_method, apply_swap_method


def _validate_swap_method(config: ConfigType) -> ConfigType:
    if not SWAP_METHOD_SCHEMA:
        return config
    return _mcuboot()[0](config)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CANopenOTAComponent),
            cv.GenerateID(canopen.CONF_CANOPEN_ID): cv.use_id(canopen.CANopenComponent),
            # reboot into the new image when the program is started
            cv.Optional(CONF_REBOOT, default=True): cv.boolean,
            **SWAP_METHOD_SCHEMA,
        }
    )
    .extend(BASE_OTA_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    _validate_swap_method,
)


def _final_validate(config: ConfigType) -> ConfigType:
    """On Zephyr the backend installs a test upgrade, which MCUboot keeps only once the image is confirmed, and
    safe_mode is what confirms it: without it every update is rolled back at the next reset."""
    # the program download objects are in the node's OD
    canopen.require_node_id(config[canopen.CONF_CANOPEN_ID], "ota canopen")
    safe_mode = fv.full_config.get().get("safe_mode") or {}
    if _is_zephyr() and safe_mode.get(CONF_DISABLED, False):
        _LOGGER.warning(
            "ota canopen: 'safe_mode: disabled: true' leaves updated images unconfirmed, "
            "MCUboot rolls them back at the next reset; enable safe_mode"
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await ota_to_code(var, config)
    await cg.register_component(var, config)

    hub = await cg.get_variable(config[canopen.CONF_CANOPEN_ID])
    canopen.enable_sdo_block(DEFAULT_SDO_BLOCK_SIZE)
    cg.add(var.set_canopen(hub))
    if not config[CONF_REBOOT]:
        cg.add(var.set_reboot(False))

    if SWAP_METHOD_SCHEMA and _is_zephyr():
        _mcuboot()[1](config)

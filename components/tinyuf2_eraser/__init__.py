"""Erases the TinyUF2 bootloader of a Seeed XIAO STM32C5, so MCUboot can be written in its place.

The `tinyuf2_eraser.erase` action sets BOOT_SEL to the BOOT0 pin, removes the flash write protection TinyUF2 puts on
its own pages at every boot, and erases 0x0-0x8000. It never resets: TinyUF2 is gone, so the board then waits for
BOOT + RESET to enter the STM32 ROM DFU bootloader, which writes MCUboot and the app (`bootloader: mcuboot`).
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components.stm32.const import VARIANT_STM32C5
from esphome.const import CONF_ID
from esphome.core import CORE
from esphome.types import ConfigType

DOMAIN = "tinyuf2_eraser"

tinyuf2_eraser_ns = cg.esphome_ns.namespace("tinyuf2_eraser")
TinyUf2Eraser = tinyuf2_eraser_ns.class_("TinyUf2Eraser", cg.Component)

CONFIG_SCHEMA = cv.Schema({cv.GenerateID(): cv.declare_id(TinyUf2Eraser)}).extend(
    cv.COMPONENT_SCHEMA
)


def _final_validate(config: ConfigType) -> ConfigType:
    from esphome.components.zephyr_rtos import zephyr_variant
    from esphome.components.zephyr_rtos.const import KEY_ZEPHYR_RTOS
    from esphome.components.zephyr_rtos.variants import stm32c5

    if KEY_ZEPHYR_RTOS not in CORE.data or zephyr_variant() != VARIANT_STM32C5:
        raise cv.Invalid(f"'{DOMAIN}' needs the stm32 platform with variant STM32C5")
    # The app must run from TinyUF2's slot at 0x8000, or erasing 0x0-0x8000 erases the app.
    if not stm32c5.uses_tinyuf2():
        raise cv.Invalid(
            f"'{DOMAIN}' needs a Seeed XIAO STM32C5 build started by TinyUF2 "
            "(board xiao_stm32c5, no 'bootloader: mcuboot')"
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    from esphome.components.zephyr_rtos import zephyr_add_prj_conf

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    zephyr_add_prj_conf("FLASH", True)


automation.register_apply_action(
    "tinyuf2_eraser.erase",
    automation.maybe_simple_id({cv.GenerateID(): cv.use_id(TinyUf2Eraser)}),
    automation.ApplyCall("erase()"),
)

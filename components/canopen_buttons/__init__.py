"""Wall buttons controlling entities of other CANopen nodes, on top of canopen_entities (docs/linking-nodes.md).

`canopen_buttons: lights:` - a binary sensor toggling a remote dimmable light on a short press and ramping its
brightness while held (LightButton, light_button.h). `covers:` - open / close buttons moving a remote cover while
held (CoverButton, cover_button.h). The remote entities are resolved and their states read through
canopen_entities' RemoteTarget (Python and C++): remote state slots, commands through the OD writer, nothing needed
on the remote node.
"""

import esphome.codegen as cg
from esphome.components import binary_sensor, canopen_entities
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_STATE
from esphome.core import CORE
from esphome.types import ConfigType

CODEOWNERS = ["@mrk-its"]
DEPENDENCIES = ["canopen_entities"]

CONF_LIGHTS = "lights"
CONF_BUTTON = "button"
CONF_HOLD_TIME = "hold_time"
CONF_RAMP_TIME = "ramp_time"
CONF_STEP_INTERVAL = "step_interval"
CONF_MIN_BRIGHTNESS = "min_brightness"
CONF_BRIGHTNESS = "brightness"
CONF_COVERS = "covers"
CONF_OPEN_BUTTON = "open_button"
CONF_CLOSE_BUTTON = "close_button"
CONF_DOUBLE_CLICK_TIME = "double_click_time"

ns = cg.esphome_ns.namespace("canopen_buttons")
LightButton = ns.class_("LightButton", cg.Component)
CoverButton = ns.class_("CoverButton", cg.Component)


def _validate_ramp(config: ConfigType) -> ConfigType:
    if config[CONF_RAMP_TIME] < config[CONF_STEP_INTERVAL]:
        raise cv.Invalid(
            f"'{CONF_RAMP_TIME}' must be at least '{CONF_STEP_INTERVAL}'",
            path=[CONF_RAMP_TIME],
        )
    return config


def _resolve_light(config: ConfigType) -> ConfigType:
    """The remote light: dimmable, its state and brightness in TPDOs (remote state slots), the OD writer on."""
    target = canopen_entities.RemoteTarget(
        "canopen_buttons light",
        config[canopen_entities.CONF_NODE],
        config[canopen_entities.CONF_ENTITY],
    )
    target.require_domain(canopen_entities.layout.DOMAIN_LIGHT)
    if CONF_BRIGHTNESS not in target.states:
        raise cv.Invalid(
            f"{target.what}: {target.node_name} '{target.entity_id}' is a light without brightness, a light button "
            "needs a dimmable light (canopen_entities.light.toggle switches it)"
        )
    target.require_od_writer()
    return {
        **config,
        canopen_entities.CONF_ENTITY: target.entity_id,
        "_target": {
            "node_id": target.node_id,
            "index": target.index,
            "old_format": target.old_format,
            "brightness_cmd": target.cmd(CONF_BRIGHTNESS),
        },
        # each marker in a dict of its own: remote_state_slots() takes one per dict
        "_state_slot": target.state_marker(CONF_STATE),
        "_brightness_slot": target.state_marker(CONF_BRIGHTNESS),
    }


# a short press (released before hold_time) toggles, holding ramps the brightness (light_button.h)
LIGHT_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LightButton),
            cv.Required(CONF_BUTTON): cv.use_id(binary_sensor.BinarySensor),
            cv.Required(canopen_entities.CONF_NODE): cv.string,
            cv.Required(canopen_entities.CONF_ENTITY): cv.string,
            cv.Optional(
                CONF_HOLD_TIME, default="400ms"
            ): cv.positive_time_period_milliseconds,
            # brightness 0 -> 100 % while held
            cv.Optional(
                CONF_RAMP_TIME, default="3s"
            ): cv.positive_time_period_milliseconds,
            # a brightness command every step_interval while ramping
            cv.Optional(CONF_STEP_INTERVAL, default="100ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=20)),
            ),
            # dimming stops here (it doesn't switch the light off); an off light starts here
            cv.Optional(CONF_MIN_BRIGHTNESS, default="2%"): cv.All(
                cv.percentage, cv.Range(min=0.004, max=0.99)
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_ramp,
    _resolve_light,
)


def _resolve_cover(config: ConfigType) -> ConfigType:
    """The remote cover, with the OD writer on (no remote state needed)."""
    target = canopen_entities.RemoteTarget(
        "canopen_buttons cover",
        config[canopen_entities.CONF_NODE],
        config[canopen_entities.CONF_ENTITY],
    )
    target.require_domain(canopen_entities.layout.DOMAIN_COVER)
    target.require_od_writer()
    return {
        **config,
        canopen_entities.CONF_ENTITY: target.entity_id,
        "_target": {
            "node_id": target.node_id,
            "index": target.index,
            "old_format": target.old_format,
        },
    }


# open / close buttons: the cover moves while one is held and stops on release (cover_button.h)
COVER_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CoverButton),
            cv.Optional(CONF_OPEN_BUTTON): cv.use_id(binary_sensor.BinarySensor),
            cv.Optional(CONF_CLOSE_BUTTON): cv.use_id(binary_sensor.BinarySensor),
            # a double-click moves the cover all the way (0ms: off)
            cv.Optional(
                CONF_DOUBLE_CLICK_TIME, default="400ms"
            ): cv.positive_time_period_milliseconds,
            cv.Required(canopen_entities.CONF_NODE): cv.string,
            cv.Required(canopen_entities.CONF_ENTITY): cv.string,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.has_at_least_one_key(CONF_OPEN_BUTTON, CONF_CLOSE_BUTTON),
    _resolve_cover,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(canopen_entities.CONF_MAPPER_ID): cv.use_id(
            canopen_entities.CANopenEntities
        ),
        cv.Optional(CONF_LIGHTS, default=[]): cv.ensure_list(LIGHT_SCHEMA),
        cv.Optional(CONF_COVERS, default=[]): cv.ensure_list(COVER_SCHEMA),
    }
)


def _slot(marker: dict) -> int:
    return canopen_entities.remote_state_slot(
        CORE.config, marker[canopen_entities.CONF_REMOTE_STATE]
    )


async def to_code(config: ConfigType) -> None:
    mapper = await cg.get_variable(config[canopen_entities.CONF_MAPPER_ID])
    if config[CONF_LIGHTS]:
        cg.add_define("USE_CANOPEN_BUTTONS_LIGHTS")
    for light in config[CONF_LIGHTS]:
        var = cg.new_Pvariable(light[CONF_ID])
        await cg.register_component(var, light)
        target = light["_target"]
        cg.add(
            var.set_remote_target(
                mapper, target["node_id"], target["index"], target["old_format"]
            )
        )
        cg.add(var.set_brightness_cmd(target["brightness_cmd"]))
        cg.add(var.set_button(await cg.get_variable(light[CONF_BUTTON])))
        cg.add(
            var.set_slots(_slot(light["_state_slot"]), _slot(light["_brightness_slot"]))
        )
        interval = light[CONF_STEP_INTERVAL].total_milliseconds
        cg.add(
            var.set_timing(
                light[CONF_HOLD_TIME].total_milliseconds,
                interval,
                interval / light[CONF_RAMP_TIME].total_milliseconds,
                light[CONF_MIN_BRIGHTNESS],
            )
        )

    if config[CONF_COVERS]:
        cg.add_define("USE_CANOPEN_BUTTONS_COVERS")
    for cover in config[CONF_COVERS]:
        var = cg.new_Pvariable(cover[CONF_ID])
        await cg.register_component(var, cover)
        target = cover["_target"]
        cg.add(
            var.set_remote_target(
                mapper, target["node_id"], target["index"], target["old_format"]
            )
        )
        if CONF_OPEN_BUTTON in cover:
            cg.add(var.set_open_button(await cg.get_variable(cover[CONF_OPEN_BUTTON])))
        if CONF_CLOSE_BUTTON in cover:
            cg.add(
                var.set_close_button(await cg.get_variable(cover[CONF_CLOSE_BUTTON]))
            )
        cg.add(
            var.set_double_click_time(cover[CONF_DOUBLE_CLICK_TIME].total_milliseconds)
        )

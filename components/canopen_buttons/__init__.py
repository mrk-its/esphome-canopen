"""Wall buttons controlling entities of other CANopen nodes, on top of canopen_entities (docs/linking-nodes.md).

`canopen_buttons: lights:` - a binary sensor toggling a remote dimmable light on a short press and ramping its
brightness while held (LightButton, light_button.h). The remote entities are resolved and their states read through
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
CONF_BINARY_SENSOR = "binary_sensor"
CONF_HOLD_TIME = "hold_time"
CONF_RAMP_TIME = "ramp_time"
CONF_STEP_INTERVAL = "step_interval"
CONF_MIN_BRIGHTNESS = "min_brightness"
CONF_BRIGHTNESS = "brightness"

ns = cg.esphome_ns.namespace("canopen_buttons")
LightButton = ns.class_("LightButton", cg.Component)


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
            cv.Required(CONF_BINARY_SENSOR): cv.use_id(binary_sensor.BinarySensor),
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

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(canopen_entities.CONF_MAPPER_ID): cv.use_id(
            canopen_entities.CANopenEntities
        ),
        cv.Optional(CONF_LIGHTS, default=[]): cv.ensure_list(LIGHT_SCHEMA),
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
        cg.add(var.set_binary_sensor(await cg.get_variable(light[CONF_BINARY_SENSOR])))
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

"""Tests for components/canopen/layout.py (run: pytest test/)."""

import importlib.util
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location(
    "canopen_layout",
    Path(__file__).parent.parent / "components" / "canopen" / "layout.py",
)
layout = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(layout)


def light_sizes(platform, **conf):
    caps = layout.entity_caps(layout.DOMAIN_LIGHT, {"platform": platform, **conf})
    return caps, layout.entity_state_sizes(layout.DOMAIN_LIGHT, caps)


@pytest.mark.parametrize(
    "platform, conf, caps, sizes",
    [
        ("binary", {}, 1, [1]),
        ("monochromatic", {}, 2, [1, 1]),
        ("cwww", {}, 4, [1, 1, 1]),
        ("color_temperature", {}, 4, [1, 1, 1]),
        ("hbridge", {}, 4, [1, 1, 1]),
        ("rgb", {}, 2, [1, 1]),
        ("rgbw", {}, 2, [1, 1]),
        ("rgbw", {"color_interlock": True}, 2, [1, 1]),
        ("rgbww", {}, 4, [1, 1, 1]),
        ("rgbww", {"color_interlock": True}, 4, [1, 1, 1]),
        ("rgbct", {}, 4, [1, 1, 1]),
        ("rgbct", {"color_interlock": True}, 4, [1, 1, 1]),
    ],
)
def test_light_platforms(platform, conf, caps, sizes):
    assert light_sizes(platform, **conf) == (caps, sizes)


@pytest.mark.parametrize(
    "platform, conf, caps",
    [
        ("time_based", {}, 1),
        ("current_based", {}, 1),
        ("endstop", {}, 1),
        ("feedback", {}, 1),
        ("template", {}, 0),
        ("template", {"has_position": True}, 1),
        ("template", {"position_action": []}, 1),
        ("template", {"tilt_action": []}, 2),
        ("template", {"tilt_lambda": "..."}, 0),
        ("venetian_blinds", {}, 3),
    ],
)
def test_cover_platforms(platform, conf, caps):
    assert layout.entity_caps(layout.DOMAIN_COVER, {"platform": platform, **conf}) == caps
    assert len(layout.entity_state_sizes(layout.DOMAIN_COVER, caps)) == 1 + bin(caps).count("1")


def test_unknown_platform_requires_caps():
    with pytest.raises(layout.LayoutError, match="caps"):
        layout.entity_caps(layout.DOMAIN_LIGHT, {"platform": "neopixelbus"})
    caps = layout.entity_caps(
        layout.DOMAIN_LIGHT, {"platform": "neopixelbus"}, ["on_off", "brightness"]
    )
    assert caps == 3
    assert layout.entity_state_sizes(layout.DOMAIN_LIGHT, caps) == [1, 1]


def test_caps_override_validation():
    with pytest.raises(layout.LayoutError, match="Unknown light capability"):
        layout.entity_caps(layout.DOMAIN_LIGHT, {"platform": "cwww"}, ["tilt"])
    with pytest.raises(layout.LayoutError, match="not supported"):
        layout.entity_caps(layout.DOMAIN_SENSOR, {"platform": "tmp102"}, ["tilt"])


def test_sensor_sizes():
    assert layout.entity_state_sizes(layout.DOMAIN_SENSOR) == [4]
    assert layout.entity_state_sizes(layout.DOMAIN_SENSOR, size=2) == [2]
    assert layout.entity_state_sizes(layout.DOMAIN_NUMBER, size=1) == [1]
    with pytest.raises(layout.LayoutError):
        layout.entity_state_sizes(layout.DOMAIN_SENSOR, size=3)


def offsets(mapping, tpdo):
    """{(entity_od_index, sub): byte offset} of one TPDO."""
    result, offset = {}, 0
    for word in mapping[tpdo]:
        result[(word >> 16, (word >> 8) & 0xFF)] = offset
        offset += (word & 0xFF) // 8
    return result


def test_dimmer2_layout():
    """dimmer2: light1-2 monochromatic, light3-5 cwww, tmp102 sensor."""
    entities = [
        (1, 0, "monochromatic"),
        (2, 0, "monochromatic"),
        (3, 0, "cwww"),
        (4, 1, "cwww"),
        (5, 1, "cwww"),
    ]
    states = []
    for index, tpdo, platform in entities:
        _, sizes = light_sizes(platform)
        states += [(index, tpdo, size) for size in sizes]
    states += [(6, 2, size) for size in layout.entity_state_sizes(layout.DOMAIN_SENSOR)]
    mapping = layout.tpdo_mapping(states)

    # offsets used by stm32-can-node-6/8/9 (light1 @0, light2 @2) and node-7 style cwww pairs
    assert offsets(mapping, 0) == {
        (0x2011, 1): 0, (0x2011, 2): 1,
        (0x2021, 1): 2, (0x2021, 2): 3,
        (0x2031, 1): 4, (0x2031, 2): 5, (0x2031, 3): 6,
    }
    assert offsets(mapping, 1)[(0x2051, 1)] == 3
    assert mapping[2] == [layout.co_link(0x2061, 1, 32)]
    assert all(not entries for entries in mapping[3:])


def test_unmapped_states_keep_subindex():
    mapping = layout.tpdo_mapping([(1, -1, 1), (1, 0, 1)])
    assert mapping[0] == [layout.co_link(0x2011, 2, 8)]


def test_tpdo_overflow():
    with pytest.raises(layout.LayoutError, match="exceeds 8 bytes"):
        layout.tpdo_mapping([(1, 0, 4), (2, 0, 4), (3, 0, 1)])


def test_layout_hash():
    empty = [[] for _ in range(layout.TPDO_COUNT)]
    assert layout.layout_hash(empty) == layout.fnv1a_32(bytes(4 * layout.TPDO_COUNT))
    # FNV-1a 32 reference value
    assert layout.fnv1a_32(b"a") == 0xE40C292C
    one = layout.tpdo_mapping([(1, 0, 1)])
    other = layout.tpdo_mapping([(1, 1, 1)])
    assert layout.layout_hash(one) != layout.layout_hash(other)


def test_layout_table():
    states = [
        (1, 0, 1, "light1.state"),
        (1, 0, 1, "light1.brightness"),
        (2, -1, 1, "sw.state"),
        (3, 4, 4, "temp.value"),
    ]
    assert layout.layout_table(states, 0x36) == [
        "TPDO0 (COB-ID 0x1b6, 2 B): [0] light1.state, [1] light1.brightness",
        "TPDO4 (COB-ID 0x236, 4 B): [0..3] temp.value",
    ]
    # labels don't change the mapping
    assert layout.tpdo_mapping(states) == layout.tpdo_mapping([s[:3] for s in states])


def climate_caps(*names):
    return layout.entity_caps(layout.DOMAIN_CLIMATE, {"platform": "thermostat"}, list(names))


def test_climate_caps():
    # no platform table: nothing optional unless listed in `caps:`
    assert layout.entity_caps(layout.DOMAIN_CLIMATE, {"platform": "thermostat"}) == 0
    assert layout.entity_caps(layout.DOMAIN_CLIMATE, {"platform": "unknown_platform"}) == 0
    assert climate_caps("two_point") == layout.CLIMATE_CAP_TWO_POINT
    assert climate_caps("fan_mode", "preset", "swing_mode") == 2 | 4 | 8
    with pytest.raises(layout.LayoutError, match="Unknown climate capability"):
        climate_caps("humidity")


@pytest.mark.parametrize(
    "caps, size, states",
    [
        (
            0,
            None,
            [("mode", 1), ("target_temperature", 2), ("current_temperature", 2), ("action", 1)],
        ),
        (
            climate_caps("two_point"),
            1,
            [
                ("mode", 1),
                ("target_temperature_low", 1),
                ("target_temperature_high", 1),
                ("current_temperature", 1),
                ("action", 1),
            ],
        ),
        (
            climate_caps("two_point", "fan_mode", "preset", "swing_mode"),
            4,
            [
                ("mode", 1),
                ("target_temperature_low", 4),
                ("target_temperature_high", 4),
                ("fan_mode", 1),
                ("preset", 1),
                ("swing_mode", 1),
                ("current_temperature", 4),
                ("action", 1),
            ],
        ),
    ],
)
def test_climate_states(caps, size, states):
    assert layout.entity_states(layout.DOMAIN_CLIMATE, caps, size) == states
    # commands: the writable states, which come first
    commands = layout.entity_commands(layout.DOMAIN_CLIMATE, caps, size)
    assert commands == states[:-2]
    assert [name for name, _ in states[-2:]] == ["current_temperature", "action"]


def test_climate_sizes_and_tpdo():
    with pytest.raises(layout.LayoutError, match="Unsupported climate size: 3"):
        layout.entity_states(layout.DOMAIN_CLIMATE, 0, 3)
    # core: 1 + 2 + 2 + 1 bytes, two-point: 1 + 2 + 2 + 2 + 1
    core = [(1, 0, s) for s in layout.entity_state_sizes(layout.DOMAIN_CLIMATE, 0)]
    two_point = [(2, 1, s) for s in layout.entity_state_sizes(layout.DOMAIN_CLIMATE, climate_caps("two_point"))]
    mapping = layout.tpdo_mapping(core + two_point)
    assert sum(word & 0xFF for word in mapping[0]) == 6 * 8
    assert sum(word & 0xFF for word in mapping[1]) == 8 * 8
    with pytest.raises(layout.LayoutError, match="exceeds 8 bytes"):
        caps = climate_caps("two_point", "fan_mode")
        layout.tpdo_mapping([(1, 0, s) for s in layout.entity_state_sizes(layout.DOMAIN_CLIMATE, caps)])


def test_entity_commands_of_other_domains():
    assert layout.entity_commands(layout.DOMAIN_COVER, 3) == layout.entity_states(layout.DOMAIN_COVER, 3)
    assert layout.entity_commands(layout.DOMAIN_SENSOR, None, 2) == [("value", 2)]


def test_scale_range():
    assert layout.scale_range(layout.DOMAIN_CLIMATE, {}) == (-40.0, 85.0)
    assert layout.scale_range(layout.DOMAIN_CLIMATE, {"size": 1, "min_value": 5, "max_value": 35}) == (5, 35)
    assert layout.scale_range(layout.DOMAIN_SENSOR, {"size": 1}) == (0.0, 254.0)
    assert layout.scale_range(layout.DOMAIN_NUMBER, {"size": 2, "min_value": -10}) == (-10, 65534.0)


def test_climate_codes():
    # ESPHome's enum values (climate_mode.h)
    assert layout.CLIMATE_MODES.index("heat") == 3
    assert layout.CLIMATE_MODES.index("auto") == 6
    assert layout.CLIMATE_ACTIONS.index("heating") == 3
    assert layout.CLIMATE_ACTIONS.index("defrosting") == 7
    assert layout.CLIMATE_FAN_MODES.index("quiet") == 9
    assert layout.CLIMATE_PRESETS.index("activity") == 7
    assert layout.CLIMATE_SWING_MODES.index("horizontal") == 3


@pytest.mark.parametrize(
    "domain, caps, size, kinds",
    [
        (
            layout.DOMAIN_CLIMATE,
            climate_caps("fan_mode", "preset", "swing_mode"),
            None,
            [
                ("mode", 1, "climate_mode"),
                ("target_temperature", 2, "value"),
                ("fan_mode", 1, "climate_fan_mode"),
                ("preset", 1, "climate_preset"),
                ("swing_mode", 1, "climate_swing_mode"),
                ("current_temperature", 2, "value"),
                ("action", 1, "climate_action"),
            ],
        ),
        (layout.DOMAIN_LIGHT, 4, None, [("state", 1, "bool"), ("brightness", 1, "percentage"), ("color_temp", 1, "color_temp")]),
        (layout.DOMAIN_SWITCH, None, None, [("state", 1, "bool")]),
        (layout.DOMAIN_BINARY_SENSOR, None, None, [("state", 1, "bool")]),
        (layout.DOMAIN_COVER, 2, None, [("state", 1, "cover_state"), ("tilt", 1, "percentage")]),
        (layout.DOMAIN_ALARM, None, None, [("state", 1, "alarm_state")]),
        (layout.DOMAIN_SENSOR, None, None, [("value", 4, "value")]),
        (layout.DOMAIN_NUMBER, None, 2, [("value", 2, "value")]),
    ],
)
def test_state_kinds(domain, caps, size, kinds):
    assert layout.entity_state_kinds(domain, caps, size) == kinds
    assert layout.entity_states(domain, caps, size) == [(n, s) for n, s, _ in kinds]
    assert all(kind in layout.KIND_CODES for _, _, kind in kinds)

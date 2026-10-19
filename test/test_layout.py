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
        ("rgb", {}, 0, [1]),
        ("rgbw", {}, 0, [1]),
        ("rgbw", {"color_interlock": True}, 0, [1]),
        ("rgbww", {}, 0, [1]),
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

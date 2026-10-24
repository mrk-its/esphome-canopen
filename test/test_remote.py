"""Tests for components/canopen/remote.py (run: pytest test/, needs ESPHome installed)."""

import sys
from pathlib import Path

import pytest

pytest.importorskip("esphome")

TEST_DIR = Path(__file__).parent
sys.path.insert(0, str(TEST_DIR.parent / "components"))

from canopen import layout, remote  # noqa: E402


def test_find_entity_top_level_and_nested():
    config = {
        "canopen": {"entities": [{"id": "light1", "index": 1}]},
        "binary_sensor": [
            {
                "platform": "gpio",
                "id": "button",
                "on_press": [{"light.turn_on": {"id": "light1"}}],
            }
        ],
        "light": [{"platform": "cwww", "id": "light1"}],
        "sensor": [{"platform": "debug", "free": {"id": "heap"}}],
    }
    # not the canopen entry nor the automation reference in binary_sensor
    assert remote.find_entity(config, "light1") == ("light", config["light"][0])
    assert remote.find_entity(config, "heap") == ("sensor", config["sensor"][0])
    with pytest.raises(layout.LayoutError, match="not declared"):
        remote.find_entity(config, "missing")


def test_normalize_substituted_values():
    canopen = remote._normalize_node_config(
        {
            "node_id": "0x36",
            "entities": [
                {"id": "a", "index": "1", "tpdo": "0"},
                {"id": "b", "index": 2, "tpdo": {"number": "1"}, "size": "2"},
            ],
        }
    )
    assert canopen["node_id"] == 0x36
    assert canopen["entities"][0] == {"id": "a", "index": 1, "tpdo": 0}
    assert canopen["entities"][1]["tpdo"] == {"number": 1}
    assert canopen["entities"][1]["size"] == 2


def test_flag():
    assert layout.flag({"x": "true"}, "x") and layout.flag({"x": True}, "x")
    assert not layout.flag({"x": "false"}, "x") and not layout.flag({}, "x")


@pytest.fixture
def node():
    return remote.get_remote_node("dimmer", TEST_DIR / "remote_node.yaml")


def test_remote_node_layout(node):
    assert node.node_id == 0x36  # from a substitution
    assert node.state(entity_id="light_hall") == {
        "node_id": 0x36,
        "index": 1,
        "entity": "light_hall",
        "state": "state",
        "tpdo": 0,
        "offset": 0,
        "size": 1,
        "domain": "light",
        "kind": "bool",
        "cmd": 0,
    }
    assert node.state(entity_id="light_table")["offset"] == 3  # after cwww light_hall
    assert node.state(entity_id="light_table", state="brightness")["offset"] == 4
    assert node.state(index=3)["tpdo"] == 1
    temperature = node.state(entity_id="temperature")
    assert (temperature["state"], temperature["tpdo"], temperature["size"]) == ("value", 2, 4)
    assert "min_value" not in temperature  # REAL32, not scaled


def test_remote_node_kinds(node):
    tilt = node.state(entity_id="cover_hall", state="tilt")
    assert (tilt["domain"], tilt["kind"], tilt["cmd"], tilt["offset"]) == ("cover", "percentage", 2, 2)
    assert node.state(entity_id="alarm_house")["kind"] == "alarm_state"
    setpoint = node.state(entity_id="setpoint")
    assert (setpoint["size"], setpoint["min_value"], setpoint["max_value"]) == (1, 10, 30)
    assert node.domain(entity_id="pump") == "switch"
    assert node.state_names(entity_id="light_table") == ["state", "brightness"]


def test_remote_node_errors(node):
    with pytest.raises(remote.RemoteError, match="not exposed"):
        node.state(entity_id="out1")
    with pytest.raises(remote.RemoteError, match="has no state 'tilt'"):
        node.state(entity_id="light_hall", state="tilt")
    with pytest.raises(remote.RemoteError, match="not sent in any TPDO"):
        node.state(entity_id="pump")


# --- remote state slots and actions (__init__.py) ---

import esphome.config_validation as cv  # noqa: E402
from esphome import automation  # noqa: E402
from esphome.core import CORE, ID  # noqa: E402

import canopen  # noqa: E402


def _action(name, node="dimmer", **conf):
    return {name: {"id": ID("can", is_manual=True), "node": node, **conf}}


def test_remote_state_slots():
    slot = canopen.CONF_REMOTE_STATE
    config = {
        "binary_sensor": [
            {
                "platform": "gpio",
                "id": "in1",
                "on_press": [
                    {"then": [{"canopen.light.toggle": {"id": "can", slot: ["dimmer", "light_table", "state"]}}]},
                    {"then": [{"canopen.light.toggle": {"id": "can", slot: ["dimmer", "light_table", "state"]}}]},
                ],
            },
            {"platform": "canopen", "id": "x", "canopen_id": "can", slot: ["dimmer", "cover_hall", "state"]},
            {"platform": "canopen", "id": "y", "canopen_id": "other", slot: ["dimmer", "relay", "state"]},
        ],
        "sensor": [
            {"platform": "canopen", "id": "z", "canopen_id": "can", slot: ["a_node", "e", "value"]},
        ],
    }
    # deduplicated, sorted, per canopen component
    assert canopen.remote_state_slots(config, "can") == [
        ("a_node", "e", "value"),
        ("dimmer", "cover_hall", "state"),
        ("dimmer", "light_table", "state"),
    ]
    assert canopen.remote_state_slot(config, "can", ["dimmer", "light_table", "state"]) == 2
    assert canopen.remote_state_slots(config, "other") == [("dimmer", "relay", "state")]


def test_rpdo_plan_with_slots(node):
    config = {
        "entities": [
            {"index": 16, "rpdo": [{**node.state(entity_id="light_hall"), "cmd": 0}]},
        ]
    }
    slots = [
        (0, {**node.state(entity_id="light_table"), "node": "dimmer"}),
        (1, {**node.state(entity_id="light_hall", state="color_temp"), "node": "dimmer"}),
    ]
    assert canopen.rpdo_plan(config, slots) == [
        (0x36, 0, [("cmd", 16, 0, 1), ("dummy", 1), ("slot", 1, 1), ("slot", 0, 1)])
    ]
    # the same remote state mapped by rpdo and by a slot
    slots = [(0, {**node.state(entity_id="light_hall"), "node": "dimmer"})]
    with pytest.raises(cv.Invalid, match="platform: canopen"):
        canopen.rpdo_plan(config, slots)


@pytest.fixture
def raw_config():
    CORE.config_path = TEST_DIR / "config.yaml"
    CORE.raw_config = {"canopen": {"node_id": 1, "remote_nodes": {"dimmer": "remote_node.yaml"}}}
    yield
    CORE.raw_config = None


def _validate(name, **conf):
    return automation.ACTION_REGISTRY[name].schema(_action(name, **conf)[name])


def test_actions(raw_config):
    toggle = _validate("canopen.cover.toggle", entity="cover_hall")
    assert toggle[canopen.CONF_REMOTE_STATE] == ["dimmer", "cover_hall", "state"]
    assert (toggle["node_id"], toggle["index"]) == (0x36, 5)
    assert [w["value"] for w in toggle["_writes"]] == [canopen.TOGGLE_COVER]

    turn_on = _validate("canopen.light.turn_on", entity="light_hall", brightness="50%", color_temperature="250 mireds")
    assert [(w["cmd"], w["encoding"], w["value"]) for w in turn_on["_writes"]] == [
        (1, canopen.ENC_PERCENTAGE, 0.5),
        (2, canopen.ENC_COLOR_TEMP, 250),
        (0, canopen.ENC_RAW, 1),
    ]
    assert canopen.CONF_REMOTE_STATE not in turn_on

    control = _validate("canopen.cover.control", entity="cover_hall", tilt=1.0)
    assert [(w["cmd"], w["value"]) for w in control["_writes"]] == [(2, 1.0)]

    number = _validate("canopen.number.set", index=7, value=20)
    assert [(w["size"], w["encoding"], w["min_value"], w["max_value"]) for w in number["_writes"]] == [
        (1, canopen.ENC_SCALED, 10, 30)
    ]
    # commands only: the remote entity doesn't need a TPDO
    assert _validate("canopen.switch.turn_on", entity="pump")["index"] == 9


@pytest.mark.parametrize(
    "name, conf, error",
    [
        ("canopen.cover.open", {"entity": "light_hall"}, "is a light, expected cover"),
        ("canopen.light.turn_on", {"entity": "relay", "brightness": 0.5}, "has no brightness"),
        ("canopen.light.turn_on", {"entity": "light_table", "color_temperature": "250 mireds"}, "has no color_temp"),
        ("canopen.switch.toggle", {"entity": "pump"}, "not sent in any TPDO"),
        ("canopen.cover.control", {"entity": "cover_hall"}, "set 'stop', 'position' or 'tilt'"),
        ("canopen.number.set", {"entity": "temperature", "value": 1}, "is a sensor, expected number"),
        ("canopen.cover.stop", {"entity": "nope"}, "not exposed"),
        ("canopen.cover.stop", {"node": "unknown", "entity": "x"}, "Unknown node 'unknown'"),
    ],
)
def test_action_errors(raw_config, name, conf, error):
    with pytest.raises(cv.Invalid, match=error):
        _validate(name, **conf)


@pytest.mark.parametrize(
    "platform, kinds, conf, error",
    [
        ("binary_sensor", ["bool"], {"entity": "cover_hall"}, "can't show dimmer cover_hall.state"),
        ("text_sensor", ["bool", "cover_state"], {"entity": "light_hall", "state": "brightness"}, "percentage"),
        ("sensor", list(layout.KIND_CODES), {"entity": "light_hall", "state": "tilt"}, "has no state 'tilt'"),
    ],
)
def test_remote_state_platform_errors(raw_config, platform, kinds, conf, error):
    with pytest.raises(cv.Invalid, match=error):
        canopen.validate_remote_state(platform, kinds)({"node": "dimmer", **conf})


def test_remote_state_platform(raw_config):
    config = canopen.validate_remote_state("sensor", list(layout.KIND_CODES))(
        {"node": "dimmer", "entity": "setpoint"}
    )
    assert config["state"] == "value"
    assert config[canopen.CONF_REMOTE_STATE] == ["dimmer", "setpoint", "value"]
    assert config["_remote"]["kind"] == "value"


# --- remote logs (__init__.py) ---


def _logs_node(block_size=3, sdo_block_transfer_size=63, **remote_logs):
    remote_logs = {"level": "INFO", "timeout": "never", "block_size": block_size, **remote_logs}
    return {"sdo_block_transfer_size": sdo_block_transfer_size, canopen.CONF_REMOTE_LOGS: remote_logs}


@pytest.mark.parametrize(
    "block_size, sdo_block_transfer_size",
    [(3, 63), (16, 63), (63, 63), (2, 2)],
)
def test_remote_logs_block_size_valid(block_size, sdo_block_transfer_size):
    canopen.validate_remote_logs(
        {"logger": {"level": "DEBUG"}}, [_logs_node(block_size, sdo_block_transfer_size)]
    )


def test_remote_logs_block_size_larger_than_sdo_buffer():
    with pytest.raises(cv.Invalid, match="block_size .64. can't be larger than sdo_block_transfer_size .63."):
        canopen.validate_remote_logs({"logger": {"level": "DEBUG"}}, [_logs_node(64, 63)])


def test_remote_logs_requires_logger():
    with pytest.raises(cv.Invalid, match="requires the 'logger' component"):
        canopen.validate_remote_logs({}, [_logs_node()])


def test_remote_logs_on_one_node_only():
    with pytest.raises(cv.Invalid, match="one canopen node only"):
        canopen.validate_remote_logs({"logger": {}}, [_logs_node(), _logs_node()])

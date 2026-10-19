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
    }
    assert node.state(entity_id="light_table")["offset"] == 3  # after cwww light_hall
    assert node.state(entity_id="light_table", state="brightness")["offset"] == 4
    assert node.state(index=3)["tpdo"] == 1
    temperature = node.state(entity_id="temperature")
    assert (temperature["state"], temperature["tpdo"], temperature["size"]) == ("value", 2, 4)


def test_remote_node_errors(node):
    with pytest.raises(remote.RemoteError, match="not exposed"):
        node.state(entity_id="out1")
    with pytest.raises(remote.RemoteError, match="has no state 'tilt'"):
        node.state(entity_id="light_hall", state="tilt")

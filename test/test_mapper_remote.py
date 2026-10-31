"""Tests for components/canopen_entities/remote.py (run: pytest test/, needs ESPHome for the YAML loader).

The modules are loaded without the package __init__.py, which needs the upstream canopen hub.
"""

from pathlib import Path
import sys
import types

import pytest

pytest.importorskip("esphome")

TEST_DIR = Path(__file__).parent
_pkg = types.ModuleType("mapper_pkg")
_pkg.__path__ = [str(TEST_DIR.parent / "components" / "canopen_entities")]
sys.modules["mapper_pkg"] = _pkg

from mapper_pkg import layout, remote  # noqa: E402


@pytest.fixture
def old_node():
    """A node on the old canopen-stack component (`canopen: entities`)."""
    return remote.get_remote_node("dimmer", TEST_DIR / "remote_node.yaml")


@pytest.fixture
def new_node():
    """The same entities on the canopen hub + canopen_entities, node 0x38, OD writer off."""
    return remote.get_remote_node("dimmer2", TEST_DIR / "mapper" / "remote_node_mapper.yaml")


def test_both_formats(old_node, new_node):
    assert (old_node.node_id, old_node.od_writer) == (0x36, True)
    assert (new_node.node_id, new_node.od_writer) == (0x38, False)
    assert old_node.mapping == new_node.mapping
    assert old_node.layout_hash == new_node.layout_hash
    state = new_node.state(entity_id="light_table", state="brightness")
    assert (state["node_id"], state["tpdo"], state["offset"], state["size"]) == (0x38, 0, 4, 1)


def test_node_section_errors(tmp_path):
    with pytest.raises(remote.RemoteError, match="no 'canopen:' section"):
        remote.node_section({"canopen_entities": {"entities": []}}, "x.yaml")
    with pytest.raises(remote.RemoteError, match="exactly one 'canopen:'"):
        remote.node_section({"canopen": [{"node_id": 1}, {"node_id": 2}]}, "x.yaml")
    node, od_writer = remote.node_section({"canopen": {"node_id": 5, "pdo_od_writer": "false"}}, "x.yaml")
    assert node["node_id"] == 5 and not od_writer


def test_tpdo_size(old_node):
    # TPDO 0: cwww light_hall (3 B) + monochromatic light_table (2 B); TPDO 3: cover 3 + alarm 1 + setpoint 1 + relay 1
    assert [old_node.tpdo_size(n) for n in range(4)] == [5, 3, 4, 6]


def _source(node, entity, state=None, **target):
    s = node.state(entity_id=entity, state=state)
    return {**s, "node": node.name, "tpdo_size": node.tpdo_size(s["tpdo"]), "label": f"{entity}.{s['state']}", **target}


def test_rpdo_plan(old_node):
    sources = [
        _source(old_node, "light_table", slot=0),  # offset 3
        _source(old_node, "light_hall", "color_temp", slot=1),  # offset 2
        _source(old_node, "light_hall", entity_index=16, cmd=0),  # offset 0, into our entity 16 command 1
        _source(old_node, "relay", slot=2),  # TPDO 3, offset 5 (last)
    ]
    assert remote.rpdo_plan(sources) == [
        (
            0x36,
            0,
            [
                (0x2102, 1, 8),  # light_hall.state -> 0x2002 + 16 * 16, command 1
                (remote.DUMMY_INDEX, 0, 8),  # light_hall.brightness
                (0x4001, 1, 8),
                (0x4000, 1, 8),
                (remote.DUMMY_INDEX, 0, 8),  # rest of the TPDO: light_table.brightness
            ],
        ),
        (0x36, 3, [(remote.DUMMY_INDEX, 0, 40), (0x4002, 1, 8)]),  # one dummy for 5 bytes
    ]
    words = remote.co_link_words(remote.rpdo_plan(sources)[1][2])
    assert words == [layout.co_link(5, 0, 40), layout.co_link(0x4002, 1, 8)]


def test_rpdo_plan_errors(old_node):
    with pytest.raises(remote.PlanError, match="light_hall.state overlaps light_hall.state in TPDO 0"):
        remote.rpdo_plan(
            [_source(old_node, "light_hall", slot=0), _source(old_node, "light_hall", entity_index=1, cmd=0)]
        )
    # TPDO 3 has 6 one-byte states: all of them plus gaps would fit, but not 9 entries
    many = [dict(_source(old_node, "relay", slot=n), offset=n, size=1, tpdo_size=9) for n in range(9)]
    with pytest.raises(remote.PlanError, match="needs 9 mapping entries"):
        remote.rpdo_plan(many)


def _raw(node_id: int, tpdo: int, offset: int, tpdo_size: int | None = None, **target: int) -> dict:
    """A raw `rpdo: {node_id, tpdo, offset}` source: one byte, the TPDO's size only if given."""
    label = f"node 0x{node_id:02x} TPDO {tpdo} offset {offset}"
    return {"node_id": node_id, "tpdo": tpdo, "offset": offset, "size": 1, "tpdo_size": tpdo_size, "label": label, **target}


def test_rpdo_plan_raw(old_node: remote.RemoteNode) -> None:
    # unknown TPDO size: the RPDO ends at the last mapped byte
    assert remote.rpdo_plan([_raw(0x40, 1, 2, entity_index=1, cmd=0), _raw(0x40, 1, 0, entity_index=2, cmd=1)]) == [
        (0x40, 1, [(0x2022, 2, 8), (remote.DUMMY_INDEX, 0, 8), (0x2012, 1, 8)]),
    ]
    # given TPDO size: dummy up to it
    assert remote.rpdo_plan([_raw(0x40, 1, 2, tpdo_size=6, entity_index=1, cmd=0)]) == [
        (0x40, 1, [(remote.DUMMY_INDEX, 0, 16), (0x2012, 1, 8), (remote.DUMMY_INDEX, 0, 24)]),
    ]
    # mixed with a named source of the same TPDO (TPDO 3, 6 B): the known size wins
    plan = remote.rpdo_plan([_raw(0x36, 3, 1, entity_index=1, cmd=0), _source(old_node, "relay", slot=0)])
    assert plan == [(0x36, 3, [(remote.DUMMY_INDEX, 0, 8), (0x2012, 1, 8), (remote.DUMMY_INDEX, 0, 24), (0x4000, 1, 8)])]


def test_rpdo_plan_raw_errors(old_node: remote.RemoteNode) -> None:
    with pytest.raises(remote.PlanError, match="relay.state overlaps node 0x36 TPDO 3 offset 5"):
        remote.rpdo_plan([_raw(0x36, 3, 5, entity_index=1, cmd=0), _source(old_node, "relay", slot=0)])
    with pytest.raises(remote.PlanError, match="has different sizes"):
        remote.rpdo_plan([_raw(0x36, 3, 0, tpdo_size=8, entity_index=1, cmd=0), _source(old_node, "relay", slot=0)])
    with pytest.raises(remote.PlanError, match="offset 7 .* is outside the 6 B TPDO 3"):
        remote.rpdo_plan([_source(old_node, "relay", slot=0), dict(_raw(0x36, 3, 7, entity_index=1, cmd=0), label="node 0x36 TPDO 3 offset 7 (x)")])

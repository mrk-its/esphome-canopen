"""Tests for components_next/canopen_entity_mapper (run: pytest test/).

Codegen and validation go through the ESPHome CLI of the upstream venv (`esphome compile --only-generate`,
`esphome config`), with the canopen hub from an upstream checkout (branch canopen-next) as a local external
component. Paths: $ESPHOME_UPSTREAM_VENV (default ~/repos/esphome-upstream-venv), $ESPHOME_CANOPEN_HUB (default
~/repos/esphome/esphome/components); the tests are skipped without them.
"""

import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess

import pytest
import yaml

REPO = Path(__file__).parent.parent
MAPPER = REPO / "components_next" / "canopen_entity_mapper"
VENV = Path(os.environ.get("ESPHOME_UPSTREAM_VENV", Path.home() / "repos" / "esphome-upstream-venv"))
HUB = Path(os.environ.get("ESPHOME_CANOPEN_HUB", Path.home() / "repos" / "esphome" / "esphome" / "components"))
ESPHOME = VENV / "bin" / "esphome"

_spec = importlib.util.spec_from_file_location("mapper_layout", MAPPER / "layout.py")
layout = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(layout)

needs_esphome = pytest.mark.skipif(
    not ESPHOME.is_file() or not (HUB / "canopen" / "__init__.py").is_file(),
    reason=f"needs {ESPHOME} and the canopen hub in {HUB}",
)

NODE_ID = 0x7D

BASE = f"""
external_components:
  - source: {{type: local, path: {HUB}}}
    components: [canopen]
  - source: {{type: local, path: {REPO / "components_next"}}}
    components: [canopen_entity_mapper]
esphome:
  name: mapper-test
esp8266:
  board: d1_mini
spi: {{clk_pin: GPIO14, mosi_pin: GPIO13, miso_pin: GPIO12}}
canbus:
  - {{platform: mcp2515, id: can_bus, cs_pin: GPIO15, can_id: 0, bit_rate: 125kbps}}
canopen:
  id: can_hub
  canbus_id: can_bus
  node_id: {NODE_ID}
output:
  - {{platform: template, id: out_float, type: float, write_action: []}}
sensor:
  - {{platform: template, id: temp, name: Temperature, unit_of_measurement: "°C", device_class: temperature}}
  - {{platform: template, id: level, name: Level}}
binary_sensor:
  - {{platform: template, id: door, name: Door, device_class: door}}
switch:
  - {{platform: template, id: relay, name: Relay, optimistic: true}}
number:
  - {{platform: template, id: setpoint, name: Setpoint, optimistic: true, min_value: 0, max_value: 100, step: 1}}
light:
  - {{platform: monochromatic, id: lamp, name: Lamp, output: out_float}}
cover:
  - {{platform: template, id: blind, name: Blind, optimistic: true, has_position: true}}
alarm_control_panel:
  - {{platform: template, id: home_alarm, name: Alarm}}
text_sensor:
  - {{platform: template, id: txt, name: Text}}
"""

# one entity of every domain, out of index order (codegen sorts)
ENTITIES = [
    {"id": "home_alarm", "index": 8, "tpdo": 5},
    {"id": "temp", "index": 1, "tpdo": 0},
    {"id": "level", "index": 2, "size": 1, "min_value": 0, "max_value": 100, "tpdo": 0},
    {"id": "door", "index": 3, "tpdo": 1},
    {"id": "relay", "index": 4, "tpdo": 1},
    {"id": "setpoint", "index": 5, "size": 2, "tpdo": {"number": 1}},
    {"id": "lamp", "index": 6, "tpdo": 2},
    {"id": "blind", "index": 7, "tpdo": 2},
]


def write_config(tmp_path: Path, entities: list, extra: dict | None = None, **options) -> Path:
    """BASE with the mapper (entities, options) and `extra` merged in (lists are extended)."""
    config = yaml.safe_load(BASE)
    config["canopen_entity_mapper"] = {"canopen_id": "can_hub", "entities": entities, **options}
    for key, value in (extra or {}).items():
        if isinstance(value, list):
            config[key] = config.get(key, []) + value
        else:
            config[key] = value
    path = tmp_path / "node.yaml"
    # JSON is YAML (flow style)
    path.write_text(json.dumps(config, indent=1), encoding="utf-8")
    return path


def run_esphome(tmp_path: Path, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [str(ESPHOME), *args, "node.yaml"], cwd=tmp_path, capture_output=True, text=True, timeout=300, check=False
    )


def generate(tmp_path: Path, entities: list, extra: dict | None = None, **options) -> tuple[str, str]:
    """(main.cpp, defines.h) of the config."""
    write_config(tmp_path, entities, extra, **options)
    result = run_esphome(tmp_path, "compile", "--only-generate")
    assert result.returncode == 0, result.stdout + result.stderr
    src = tmp_path / ".esphome" / "build" / "mapper-test" / "src"
    return (src / "main.cpp").read_text(encoding="utf-8"), (src / "esphome" / "core" / "defines.h").read_text(
        encoding="utf-8"
    )


def config_error(tmp_path: Path, entities: list, extra: dict | None = None, **options) -> str:
    """The output of `esphome config` for an invalid config."""
    write_config(tmp_path, entities, extra, **options)
    result = run_esphome(tmp_path, "config")
    assert result.returncode != 0, "config unexpectedly valid"
    return result.stdout + result.stderr


def test_layout_copy_identical():
    """The mapper carries a copy of the old component's layout.py; they must not drift apart while both exist."""
    old = REPO / "components" / "canopen" / "layout.py"
    assert (MAPPER / "layout.py").read_bytes() == old.read_bytes()


@needs_esphome
def test_codegen(tmp_path):
    main_cpp, defines = generate(tmp_path, ENTITIES)
    hub = "can_hub->"
    mapper = re.search(r"(\w+)->set_layout_hash\(", main_cpp).group(1) + "->"

    # the ESPHome identity and version string can2mqtt checks; set after the hub's own defaults
    identity = f"{hub}set_identity(2778335477UL, 1809818073, 1, 0);"  # 0xA59A08F5, 0x6BDFA1D9, revision 1
    assert identity in main_cpp
    assert main_cpp.index(f"{hub}set_identity(0x00, 0x00, 0x00, 0x00);") < main_cpp.index(identity)
    assert re.search(
        rf'const char \*const version_str_{NODE_ID} = "node_id: 7d, ver: " ESPHOME_VERSION "\.\d{{8}}\.\d{{6}}";',
        main_cpp,
    )
    assert f"{hub}set_software_version(version_str_{NODE_ID} + 18);" in main_cpp

    # all 8 TPDOs, unused ones without mappings (disabled); states in subindex order, TPDO by TPDO
    mapping = [
        [(0x2011, 1, 32), (0x2021, 1, 8)],
        [(0x2031, 1, 8), (0x2041, 1, 8), (0x2051, 1, 16)],
        [(0x2061, 1, 8), (0x2061, 2, 8), (0x2071, 1, 8), (0x2071, 2, 8)],
        [],
        [],
        [(0x2081, 1, 8)],
        [],
        [],
    ]
    for tpdo, entries in enumerate(mapping):
        items = ", ".join(f"{{0x{index:04X}, {sub}, {bits}}}" for index, sub, bits in entries)
        cob_id = layout.tpdo_cob_id(tpdo, NODE_ID)
        assert f"{hub}configure_tpdo({tpdo}, {cob_id}, {{{items}}});" in main_cpp
    words = [[layout.co_link(index, sub, bits) for index, sub, bits in entries] for entries in mapping]
    assert f"{mapper}set_layout_hash({layout.layout_hash(words)});" in main_cpp

    # entities in index order, with the sizes, scales and caps
    calls = [
        "add_sensor(temp, 1, 4, 0.0f, 65534.0f)",
        "add_sensor(level, 2, 1, 0.0f, 100.0f)",
        "add_binary_sensor(door, 3)",
        "add_switch(relay, 4)",
        "add_number(setpoint, 5, 2, 0.0f, 65534.0f)",
        f"add_light(lamp, 6, {layout.LIGHT_CAP_BRIGHTNESS})",
        f"add_cover(blind, 7, {layout.COVER_CAP_POSITION})",
        "add_alarm(home_alarm, 8)",
    ]
    positions = [main_cpp.index(f"{mapper}{call};") for call in calls]
    assert positions == sorted(positions)
    # the entity StaticVector
    assert "#define CANOPEN_ENTITY_MAPPER_ENTITIES 8" in defines
    # the hub's node id, the OD writer on by default
    assert f"{mapper}set_node_id({NODE_ID});" in main_cpp
    assert f"{mapper}enable_od_writer();" in main_cpp


@needs_esphome
def test_codegen_od_writer_off(tmp_path):
    """Without the OD writer TPDO 7 is free for entities."""
    main_cpp, _ = generate(tmp_path, [{"id": "temp", "index": 1, "tpdo": 7}], od_writer=False)
    assert "enable_od_writer()" not in main_cpp
    cob_id = layout.tpdo_cob_id(7, NODE_ID)
    assert f"can_hub->configure_tpdo(7, {cob_id}, {{{{0x2011, 1, 32}}}});" in main_cpp


@needs_esphome
def test_codegen_no_entities(tmp_path):
    main_cpp, defines = generate(tmp_path, [])
    for tpdo in range(layout.TPDO_COUNT):
        assert f"can_hub->configure_tpdo({tpdo}, {layout.tpdo_cob_id(tpdo, NODE_ID)}, {{}});" in main_cpp
    assert "->add_" not in main_cpp.split("set_layout_hash")[1]
    assert "#define CANOPEN_ENTITY_MAPPER_ENTITIES 1" in defines


@needs_esphome
@pytest.mark.parametrize(
    "entities, message",
    [
        pytest.param(
            [{"id": "temp", "index": 3}, {"id": "door", "index": 3}], "index 3 is used twice", id="duplicate-index"
        ),
        pytest.param([{"id": "txt", "index": 1}], "is not supported", id="unsupported-domain"),
        pytest.param([{"id": "nothing", "index": 1}], "Couldn't find ID 'nothing'", id="unknown-id"),
        pytest.param([{"id": "temp", "index": 0}], "value must be at least 1", id="index-0"),
        pytest.param(
            [
                {"id": "temp", "index": 1, "tpdo": 0},
                {"id": "setpoint", "index": 2, "tpdo": 0},
                {"id": "door", "index": 3, "tpdo": 0},
            ],
            "TPDO 0 exceeds 8 bytes",
            id="tpdo-overflow",
        ),
        pytest.param(
            [{"id": "temp", "index": 1, "caps": ["position"]}],
            "'caps' is not supported for sensor entities",
            id="caps-on-sensor",
        ),
        pytest.param(
            [{"id": "lamp", "index": 1, "caps": ["sparkle"]}], "Unknown light capability 'sparkle'", id="unknown-cap"
        ),
        pytest.param(
            [{"id": "temp", "index": 1, "size": 3}], "Unknown value '3', valid options are '1', '2', '4'", id="size-3"
        ),
        pytest.param(
            [{"id": "temp", "index": 1, "tpdo": {"number": 7}}],
            "TPDO 7 is reserved for the OD writer (od_writer: false frees it)",
            id="tpdo-7-od-writer",
        ),
        pytest.param([{"id": "temp", "index": 1, "tpdo": 8}], "value must be at most 7", id="tpdo-8"),
    ],
)
def test_invalid(tmp_path, entities, message):
    assert message in config_error(tmp_path, entities)


# ---- links to other nodes (remote_nodes) ----

REMOTE_NODES = {
    # old component (canopen: entities), node 0x36, OD writer on
    "dimmer": str(REPO / "test" / "remote_node.yaml"),
    # canopen hub + canopen_entity_mapper, node 0x38, OD writer off
    "dimmer2": str(REPO / "test" / "mapper" / "remote_node_mapper.yaml"),
}


def remote_entity(platform: str, entity_id: str, node: str, entity: str, state: str | None = None) -> dict:
    conf = {"platform": "canopen_entity_mapper", "id": entity_id, "node": node, "entity": entity}
    if state is not None:
        conf["state"] = state
    return {platform: [conf]}


def button(*actions: dict) -> dict:
    return {"button": [{"platform": "template", "id": "btn", "name": "Btn", "on_press": list(actions)}]}


@needs_esphome
def test_codegen_remote(tmp_path):
    extra = {
        **remote_entity("binary_sensor", "hall_on", "dimmer", "light_hall"),
        **remote_entity("sensor", "table_brightness", "dimmer", "light_table", "brightness"),
        **remote_entity("text_sensor", "cover_text", "dimmer", "cover_hall"),
        **button(
            {"canopen.light.toggle": {"node": "dimmer", "entity": "light_hall"}},
            {"canopen.light.turn_on": {"node": "dimmer", "entity": "light_table", "brightness": "50%"}},
            {"canopen.number.set": {"node": "dimmer", "entity": "setpoint", "value": 20}},
        ),
    }
    entities = [{"id": "relay", "index": 4, "tpdo": 1, "rpdo": [{"node": "dimmer", "entity": "relay"}]}]
    main_cpp, defines = generate(tmp_path, entities, extra, remote_nodes=REMOTE_NODES)
    mapper = re.search(r"(\w+)->set_layout_hash\(", main_cpp).group(1)

    # slots sorted by (node, entity, state): cover_hall.state, light_hall.state, light_table.brightness
    assert "#define CANOPEN_ENTITY_MAPPER_REMOTE_SLOTS 3" in defines
    for n in range(3):
        assert f"{mapper}->add_remote_state({n}, 1);" in main_cpp
    assert f"cover_text->attach({mapper}, 0, 3, 1, 0.0f, 0.0f);" in main_cpp  # kind 3: cover state
    assert f"hall_on->attach({mapper}, 1, 0, 1, 0.0f, 0.0f);" in main_cpp
    assert f"table_brightness->attach({mapper}, 2, 1, 1, 0.0f, 0.0f);" in main_cpp

    # one RPDO per remote TPDO, the TPDO's full length: TPDO 0 (hall 3 B + table 2 B), TPDO 3 (6 B)
    cob0, cob3 = layout.tpdo_cob_id(0, 0x36), layout.tpdo_cob_id(3, 0x36)
    # hall.state -> slot 1, hall brightness / color_temp + table.state as one dummy, table.brightness -> slot 2
    assert f"can_hub->configure_rpdo(0, {cob0}, {{{{0x4001, 1, 8}}, {{0x0005, 0, 24}}, {{0x4002, 1, 8}}}});" in main_cpp
    # cover_hall.state (offset 0) -> slot 0; relay.state (offset 5) into our relay's command 1 (0x2002 + 16 * 4)
    assert f"can_hub->configure_rpdo(1, {cob3}, {{{{0x4000, 1, 8}}, {{0x0005, 0, 32}}, {{0x2042, 1, 8}}}});" in main_cpp

    # actions: the target, the writes (cmd, size, encoding, min, max, value); the toggle reads slot 1
    assert main_cpp.count("->set_target(54, 1);") == 1  # light_hall
    assert "get_remote_state(1); return s ? 0.0f : 1.0f;" in main_cpp
    def write(cmd: int, size: int, encoding: int, min_value: str, max_value: str, value: str) -> bool:
        """add_write(); constant values are generated as lambdas"""
        args = f"{cmd}, {size}, {encoding}, {min_value}, {max_value}, "
        return re.search(rf"->add_write\({re.escape(args)}\[\]\(\) -> float \{{\s*return {re.escape(value)};", main_cpp)

    # turn_on: brightness (cmd 1, percentage), then state 1 (brightness alone doesn't switch a light on)
    assert "->set_target(54, 2);" in main_cpp
    assert write(1, 1, 1, "0.0f", "0.0f", "0.5f") and write(0, 1, 0, "0.0f", "0.0f", "1")
    # number.set on a scaled UINT8 (10..30)
    assert "->set_target(54, 7);" in main_cpp and write(0, 1, 3, "10", "30", "20.0f")


@needs_esphome
def test_codegen_raw_rpdo(tmp_path: Path) -> None:
    entities = [
        # lamp (6): brightness (command 1) from byte 2 of node 0x40 TPDO 1, state from byte 0
        {"id": "lamp", "index": 6, "rpdo": [{"node_id": 0x40, "tpdo": 1, "offset": 2, "cmd": 1}, {"node_id": 0x40, "tpdo": 1, "offset": 0}]},
        # relay (4): byte 3 of a 5-byte TPDO 4 of node 0x41, and byte 1 of the old dimmer's TPDO 3 (6 B, known)
        {"id": "relay", "index": 4, "rpdo": [{"node_id": 0x41, "tpdo": 4, "offset": 3, "tpdo_size": 5}]},
        {"id": "door", "index": 3, "rpdo": [{"node_id": 0x36, "tpdo": 3, "offset": 1}, {"node": "dimmer", "entity": "relay"}]},
    ]
    main_cpp, _ = generate(tmp_path, entities, remote_nodes=REMOTE_NODES)
    rpdos = re.findall(r"can_hub->configure_rpdo\((\d+), (\d+), (.*?)\);", main_cpp)
    assert [(int(n), int(cob)) for n, cob, _ in rpdos] == [
        (0, layout.tpdo_cob_id(3, 0x36)),
        (1, layout.tpdo_cob_id(1, 0x40)),
        (2, layout.tpdo_cob_id(4, 0x41)),
    ]
    # door (3) state from byte 1 and from relay.state (byte 5), the TPDO's full 6 bytes
    assert rpdos[0][2] == "{{0x0005, 0, 8}, {0x2032, 1, 8}, {0x0005, 0, 24}, {0x2032, 1, 8}}"
    # no TPDO size: ends at byte 2
    assert rpdos[1][2] == "{{0x2062, 1, 8}, {0x0005, 0, 8}, {0x2062, 2, 8}}"
    assert rpdos[2][2] == "{{0x0005, 0, 24}, {0x2042, 1, 8}, {0x0005, 0, 8}}"


@needs_esphome
@pytest.mark.parametrize(
    "entities, message",
    [
        pytest.param(
            [{"id": "temp", "index": 1, "rpdo": [{"node_id": 0x40, "tpdo": 1, "offset": 0}]}],
            "node 0x40 TPDO 1 offset 0 is 1 B, but 'temp' command 0 (value) is 4 B",
            id="raw-size",
        ),
        pytest.param(
            [{"id": "relay", "index": 4, "rpdo": [{"node_id": 0x40, "tpdo": 1, "offset": 0, "cmd": 1}]}],
            "'relay' has no command 1",
            id="raw-cmd",
        ),
        pytest.param(
            [{"id": "relay", "index": 4, "rpdo": [{"node_id": 0x40, "tpdo": 1, "offset": 4, "tpdo_size": 4}]}],
            "offset 4 is outside a 4 B TPDO",
            id="raw-tpdo-size",
        ),
        pytest.param(
            [{"id": "relay", "index": 4, "rpdo": [{"node_id": 0x40, "tpdo": 8, "offset": 0}]}],
            "value must be at most 7",
            id="raw-tpdo-range",
        ),
        pytest.param(
            [
                {"id": "relay", "index": 4, "rpdo": [{"node_id": 0x40, "tpdo": 1, "offset": 2}]},
                {"id": "door", "index": 3, "rpdo": [{"node_id": 0x40, "tpdo": 1, "offset": 2}]},
            ],
            "node 0x40 TPDO 1 offset 2 (rpdo of 'door') overlaps node 0x40 TPDO 1 offset 2 (rpdo of 'relay')",
            id="raw-overlap",
        ),
    ],
)
def test_invalid_raw_rpdo(tmp_path: Path, entities: list, message: str) -> None:
    assert message in config_error(tmp_path, entities, remote_nodes=REMOTE_NODES)


@needs_esphome
@pytest.mark.parametrize(
    "entities, extra, message",
    [
        pytest.param(
            [],
            remote_entity("sensor", "x", "nowhere", "light_hall"),
            "Unknown node 'nowhere', add it to 'remote_nodes'",
            id="unknown-node",
        ),
        pytest.param(
            [],
            remote_entity("binary_sensor", "x", "dimmer", "temperature"),
            "binary_sensor can't show dimmer temperature.value (value), supported: bool",
            id="platform-kind",
        ),
        pytest.param(
            [],
            remote_entity("sensor", "x", "dimmer", "pump"),
            "dimmer entity 'pump' is not sent in any TPDO",
            id="remote-not-in-tpdo",
        ),
        pytest.param(
            [],
            button({"canopen.cover.open": {"node": "dimmer", "entity": "light_hall"}}),
            "canopen.cover.open: dimmer 'light_hall' is a light, expected cover",
            id="action-domain",
        ),
        pytest.param(
            [],
            button({"canopen.light.turn_off": {"node": "dimmer2", "entity": "light_hall"}}),
            "canopen.light.turn_off: dimmer2 has the OD writer off",
            id="action-od-writer-off",
        ),
        pytest.param(
            [{"id": "relay", "index": 4, "rpdo": [{"node": "dimmer", "entity": "temperature"}]}],
            {},
            "dimmer temperature.value is 4 B, but 'relay' command 0 (state) is 1 B",
            id="rpdo-size",
        ),
        pytest.param(
            [{"id": "relay", "index": 4, "rpdo": [{"node": "dimmer", "entity": "light_hall"}]}],
            remote_entity("binary_sensor", "x", "dimmer", "light_hall"),
            "dimmer light_hall.state (remote state slot 0) overlaps dimmer light_hall.state (rpdo of 'relay')",
            id="slot-and-rpdo",
        ),
    ],
)
def test_invalid_remote(tmp_path, entities, extra, message):
    assert message in config_error(tmp_path, entities, extra, remote_nodes=REMOTE_NODES)


@needs_esphome
def test_codegen_remote_logs(tmp_path: Path) -> None:
    remote_logs = {"level": "DEBUG", "timeout": "never", "block_size": 8, "buffer_size": 1024}
    main_cpp, defines = generate(tmp_path, ENTITIES, {"logger": {}}, remote_logs=remote_logs)
    assert "#define USE_CANOPEN_ENTITY_MAPPER_LOGS" in defines
    assert "#define CANOPEN_ENTITY_MAPPER_LOG_BUFFER_SIZE 1024" in defines
    # block transfer for the uploads, the default download cap (the hub has no sdo_block_size)
    assert "#define CANOPEN_SDO_BLOCK_SIZE 16" in defines
    logs = re.search(r"new\((\w+)\) canopen_entity_mapper::RemoteLogs\(\);", main_cpp).group(1)
    assert f"{logs}->set_default_level(5);" in main_cpp  # DEBUG
    assert f"{logs}->set_timeout(" not in main_cpp  # never: 0
    assert f"{logs}->set_block_size(8);" in main_cpp
    # the record before the hub's setup, the logger callback before App.setup()
    assert main_cpp.index(f"{logs}->set_canopen(can_hub);") < main_cpp.index(f"{logs}->start();")
    assert main_cpp.index(f"{logs}->start();") < main_cpp.index("App.setup();")

    main_cpp, _ = generate(tmp_path, ENTITIES, {"logger": {}}, remote_logs={})
    assert re.search(r"->set_default_level\(3\);", main_cpp)  # INFO
    assert re.search(r"->set_timeout\(30000\);", main_cpp)
    assert re.search(r"->set_block_size\(3\);", main_cpp)


@needs_esphome
@pytest.mark.parametrize(
    "extra, remote_logs, message",
    [
        pytest.param({}, {}, "'remote_logs' requires the 'logger' component", id="no-logger"),
        pytest.param({"logger": {}}, {"block_size": 128}, "value must be at most 127", id="block-size"),
        pytest.param({"logger": {}}, {"timeout": "100ms"}, "remote_logs", id="timeout-short"),
        pytest.param({"logger": {}}, {"level": "LOUD"}, "Unknown value 'LOUD'", id="level"),
    ],
)
def test_invalid_remote_logs(tmp_path: Path, extra: dict, remote_logs: dict, message: str) -> None:
    assert message in config_error(tmp_path, ENTITIES, extra, remote_logs=remote_logs)

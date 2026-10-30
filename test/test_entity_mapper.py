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


def write_config(tmp_path: Path, entities: list, **options) -> Path:
    path = tmp_path / "node.yaml"
    # JSON is YAML (flow style)
    mapper = {"canopen_id": "can_hub", "entities": entities, **options}
    path.write_text(BASE + f"canopen_entity_mapper: {json.dumps(mapper)}\n", encoding="utf-8")
    return path


def run_esphome(tmp_path: Path, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [str(ESPHOME), *args, "node.yaml"], cwd=tmp_path, capture_output=True, text=True, timeout=300, check=False
    )


def generate(tmp_path: Path, entities: list, **options) -> tuple[str, str]:
    """(main.cpp, defines.h) of the config."""
    write_config(tmp_path, entities, **options)
    result = run_esphome(tmp_path, "compile", "--only-generate")
    assert result.returncode == 0, result.stdout + result.stderr
    src = tmp_path / ".esphome" / "build" / "mapper-test" / "src"
    return (src / "main.cpp").read_text(encoding="utf-8"), (src / "esphome" / "core" / "defines.h").read_text(
        encoding="utf-8"
    )


def config_error(tmp_path: Path, entities: list) -> str:
    """The output of `esphome config` for an invalid config."""
    write_config(tmp_path, entities)
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

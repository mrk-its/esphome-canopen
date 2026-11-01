# ESPHome CANopen

An [ESPHome](https://esphome.io/) external component, `canopen_entities`, that exposes an ESPHome device's entities
on a CAN bus. Lights, switches, sensors, covers and other entities become objects of a CANopen node, and the
[can2mqtt](https://github.com/mrk-its/can2mqtt) bridge brings them into Home Assistant. You get wired, reliable
home automation nodes that need no WiFi. Nodes can also control each other directly, be updated over CAN, and send
their logs over CAN.

The CANopen node itself is the `canopen` component (CiA 301 on [CANopenNode](https://github.com/CANopenNode/CANopenNode)),
which is being contributed to ESPHome. Until it is in an ESPHome release, this repository ships it too
(`components/canopen`, a copy of the [mrk-its/esphome](https://github.com/mrk-its/esphome/tree/canopen-full) branch it
is developed on), so one `external_components` entry brings both.

> **Status:** `canopen_entities` replaces this repository's old `canopen` component (built on canopen-stack). It is
> new: tested on STM32 hardware (ESPHome on Zephyr), ESP32 and ESP8266 builds compile but haven't been tested on
> hardware yet. Nodes with the old component keep working on the same bus. The old component stays available from the
> `main` branch. See [Migrating from the old component](docs/migration.md).

## How it works

```
 ┌─────────────┐   ┌─────────────┐
 │ ESPHome node│   │ ESPHome node│  ...   (canopen + canopen_entities)
 └──────┬──────┘   └──────┬──────┘
════════╧═════════════════╧═══════════ CAN bus
                 │
          ┌──────┴──────┐  MQTT   ┌────────────┐  MQTT discovery  ┌────────────────┐
          │  can2mqtt   │────────▶│ MQTT broker│─────────────────▶│ Home Assistant │
          └─────────────┘         └────────────┘                  └────────────────┘
          Linux box / HA add-on
```

* Every node has a `node_id`. Each exposed entity gets an `index` in the node's CANopen
  [Object Dictionary](OBJECT_DICTIONARY.md), together with its metadata (name, type, unit, device class).
* can2mqtt reads the metadata and creates the entities in Home Assistant automatically.
* State changes are broadcast in TPDO frames. Commands from Home Assistant are written back to the node.
* Nodes can listen to each other's TPDOs, so a wall switch can drive a light on another node without Home
  Assistant or can2mqtt in the loop.

## Features

* Entity types: `sensor`, `number`, `binary_sensor`, `switch`, `cover` (position / tilt), `light` (brightness /
  color temperature) and `alarm_control_panel`
* State broadcast in up to 8 TPDOs, with compact 1- or 2-byte encoding of sensor values
* [Linking nodes](docs/linking-nodes.md): actions such as `canopen_entities.light.toggle` and remote states as local
  entities, with other nodes referred to by their YAML file and entity ids
* [OTA over CAN](docs/ota.md): firmware updates from Home Assistant through can2mqtt
* [Remote logs](docs/remote-logs.md): read a node's log in can2mqtt, no serial cable needed
* From the `canopen` component: heartbeat monitoring of other nodes (`heartbeat_clients`, `on_node_event`), NMT,
  emergency messages, an SDO client for lambdas, several nodes on one bus
  ([`canopen` docs](https://github.com/mrk-its/esphome-docs/blob/canopen-full/src/content/docs/components/canopen.mdx))
* Runs on ESPHome for ESP32 and ESP8266, and on STM32 with ESPHome on Zephyr (`platform: zephyr`, from the
  [esphomeplatformzephyr](https://github.com/esphomeplatformzephyr/esphome) fork)

## What you need

* **ESPHome `dev`** (or 2026.11 and newer, once released): the `canopen` component uses a `canbus` API that isn't in
  ESPHome 2026.10 and older. On those the build fails with an error about `send_data`. The esphomeplatformzephyr fork
  has it.
* **Nodes**: an ESP32 board (it has a [built-in CAN controller](https://esphome.io/components/canbus/esp32/) and
  needs only an external 3.3 V CAN transceiver such as SN65HVD230), an ESP8266 with an
  [MCP2515](https://esphome.io/components/canbus/mcp2515/) module, or an STM32 board running ESPHome on Zephyr
  ([esphomeplatformzephyr](https://github.com/esphomeplatformzephyr/esphome)).
* **CAN bus**: a twisted pair with a 120 Ω terminator at each end, and the same bit rate on every node.
* **A can2mqtt host** on the bus: the [Home Assistant add-on](https://github.com/mrk-its/can2mqtt) or any Linux box
  with a [python-can](https://python-can.readthedocs.io/en/stable/interfaces.html) supported interface.
  [esphome-canbus-usb-serial](https://github.com/mrk-its/esphome-canbus-usb-serial) turns an ESP32-S3/C3 into a
  cheap USB-CAN adapter.
* **An MQTT broker** connected to Home Assistant with discovery enabled.

## Quick start

**1. Add the components to a node's YAML.** This example uses an ESP32 with a relay and a temperature sensor:

```yaml
esphome:
  name: can-node-1

esp32:
  board: esp32dev
  framework:
    type: esp-idf

external_components:
  # canopen_entities, and the canopen component until it is in an ESPHome release
  - source: github://mrk-its/esphome-canopen@canopen_entities

logger:
  logs:
    canbus: INFO      # canbus logs every frame at DEBUG

canbus:
  - platform: esp32_can
    id: can_bus
    rx_pin: GPIO22
    tx_pin: GPIO23
    can_id: 0
    bit_rate: 125kbps

canopen:
  node_id: 1          # unique on the bus, 1..127

canopen_entities:
  entities:
    - id: relay
      index: 1        # stable id of the entity, 1..255
      tpdo: 0         # broadcast state changes
    - id: temperature
      index: 2
      tpdo: 0

ota:
  - platform: canopen_entities  # later updates over CAN

switch:
  - platform: gpio
    id: relay
    name: "Relay"
    pin: GPIO2

sensor:
  - platform: internal_temperature
    id: temperature
    name: "Temperature"
```

WiFi, `api:` and `ota: platform: esphome` are optional. You can add them as on any ESPHome node, for example to
keep the node in Home Assistant through the native API as well.

**2. Flash it** the first time over USB: `esphome run can-node-1.yaml`.

**3. Run can2mqtt** connected to the same bus. Install the Home Assistant add-on, or run it standalone:

```
python3 -m venv venv
venv/bin/pip install git+https://github.com/mrk-its/can2mqtt
venv/bin/can2mqtt -i socketcan -c can0 -s <mqtt-broker-host>
```

**4. Done.** The entities show up in Home Assistant, under a device named after the node. Firmware updates can now
go over CAN, see [OTA over CAN](docs/ota.md).

A few rules:
* `index` and `node_id` together identify the entity in Home Assistant. Don't renumber them after the node is in use.
* TPDOs 0..6 are free to use; TPDO 7 is reserved for node-to-node commands (unless `od_writer: false`). Each TPDO
  holds 8 bytes. The build fails if the states don't fit, and it logs the computed layout.
* The `canopen` component from this repository replaces ESPHome's own, also once ESPHome has one. To use ESPHome's
  (when it is complete in your release), load only `components: [canopen_entities]` from here; with an incomplete
  one `canopen_entities` reports what is missing.

## Documentation

| | |
|---|---|
| [Configuration](docs/configuration.md) | all `canopen_entities:` options, entity / TPDO / RPDO schemas, the `canopen:` options that matter here |
| [Linking nodes](docs/linking-nodes.md) | `remote_nodes`, `canopen_entities.*` actions, `platform: canopen_entities` remote-state entities |
| [OTA over CAN](docs/ota.md) | firmware updates through can2mqtt, Zephyr / MCUboot |
| [Remote logs](docs/remote-logs.md) | node logs over CAN (`remote_logs:`) |
| [Migrating from the old component](docs/migration.md) | old `canopen:` options and where they went |
| [`canopen` component](https://github.com/mrk-its/esphome-docs/blob/canopen-full/src/content/docs/components/canopen.mdx) | the CANopen node: heartbeats, NMT, SDO, PDOs (draft of its esphome.io page) |
| [Object Dictionary](OBJECT_DICTIONARY.md) | OD layout, entity types, TPDO layout, for integrators |
| [Changelog](CHANGELOG.md) | |

## Troubleshooting

* **Nothing shows up in Home Assistant**: check that all nodes and the can2mqtt interface use the same bit rate,
  that the bus is terminated, and that MQTT discovery is enabled. can2mqtt logs a `node_id: .., device name: ..`
  line for every node it detects.
* **Build fails with an error about `send_data`**: your ESPHome is too old, see [What you need](#what-you-need).
* **`canopen` component lacks enable_pdo, ...**: the `canopen` component of your ESPHome isn't complete yet; load
  `canopen` from this repository too (drop the `components:` filter, or list `[canopen, canopen_entities]`).
* **Node gets slow or drops frames**: at `DEBUG` the `canbus` component logs every received frame. Over a serial
  console that is slower than the bus, so set `logger: logs: canbus: INFO`.
* **OTA over CAN is very slow (seconds per KB)**: frames of an SDO block are being lost, and every loss costs a
  2 s timeout. Lower `sdo_block_size` on `canopen:` (e.g. 16, the default with `canopen_entities`), or on
  `zephyr_can` raise `rx_buffer_size` to at least the block size.
* **`rx errors` on `zephyr_can`** point at the physical bus: stub length, termination, wiring.

## Support

* [Discord server](https://discord.gg/VXjUSnUWsd)
* [GitHub issues](https://github.com/mrk-its/esphome-canopen/issues)

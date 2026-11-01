# Migrating from the old component
The old `canopen:` component of this repository (on [canopen-stack](https://github.com/mrk-its/canopen-stack)) is
replaced by two: the `canopen` component (the CANopen node on [CANopenNode](https://github.com/CANopenNode/CANopenNode),
being contributed to ESPHome) and `canopen_entities` from this repository. The old component is still available from
the `main` branch of this repository.

The wire format is the same ([OBJECT_DICTIONARY.md](../OBJECT_DICTIONARY.md)): object dictionary layout, TPDO layout,
OD writer and OTA. So:
* nodes on the old and the new component work together on one bus, and link to each other (`remote_nodes` reads both
  YAML formats);
* can2mqtt handles both;
* with the same `node_id` and entity `index` values the entities keep their identity in Home Assistant.

A node can be moved to the new component with an OTA update over CAN (same upload protocol); this hasn't been tested
on hardware yet, so keep a way to flash it over USB.

## YAML
```yaml
# old
external_components:
  - source: github://mrk-its/esphome-canopen

canopen:
  node_id: 0x10
  sdo_block_transfer_size: 16
  entities:
    - {id: relay, index: 1, tpdo: 0}
  remote_nodes:
    dimmer: dimmer.yaml

ota:
  - platform: canopen
```

```yaml
# new
external_components:
  - source: github://mrk-its/esphome@canopen-full   # until canopen is in an ESPHome release
    components: [canopen]
  - source: github://mrk-its/esphome-canopen@feature/entity-mapper
    components: [canopen_entities]

canopen:
  node_id: 0x10
  sdo_block_size: 16

canopen_entities:
  entities:
    - {id: relay, index: 1, tpdo: 0}
  remote_nodes:
    dimmer: dimmer.yaml

ota:
  - platform: canopen_entities
```

ESPHome `dev` (2026.11 and newer once released) is required, see [What you need](../README.md#what-you-need).

## Options
| Old (`canopen:`) | New |
|---|---|
| `node_id`, `canbus_id`, `heartbeat_interval` | `canopen:`, unchanged |
| `entities` | `canopen_entities: entities:`; `index` 1..255 (was 1..64) |
| `remote_nodes`, `remote_logs` | `canopen_entities:`, same options ([remote logs](remote-logs.md): `block_size` is no longer limited by the SDO block size) |
| `pdo_od_writer` | `canopen_entities: od_writer:` |
| `sdo_block_transfer_size` (default 63) | `canopen: sdo_block_size:` (16 if not set) |
| `hw_version` | `canopen: hardware_version:` |
| `sw_version` (ignored) | dropped: the software version is set by `canopen_entities` |
| `heartbeat_clients` | `canopen: heartbeat_clients:`, same `node_id` / `timeout` |
| `on_hb_consumer_event` (`node_id`) | `canopen: on_node_event:` (`node_id`, `event`: online / offline / NMT changed / reset, `state`) |
| `on_pre_operational`, `on_operational` | `canopen: on_nmt_state_change:` (`state`) |
| `tpdo: {immediate: ...}` | accepted, no effect: every change is sent at the next CANopen processing |
| raw `rpdo: {node_id, tpdo, offset, cmd}` | the same; the command must be 1 byte, new `tpdo_size` ([RPDO schema](configuration.md#rpdo-schema)) |
| `platform: canopen` (`binary_sensor`, `sensor`, `text_sensor`) | `platform: canopen_entities` |
| `canopen.light.toggle`, `canopen.<domain>.<action>` | `canopen_entities.light.toggle`, `canopen_entities.<domain>.<action>` |
| `ota: - platform: canopen` | `ota: - platform: canopen_entities` |
| several `canopen:` nodes in one firmware | several `canopen:` nodes are possible, but one `canopen_entities:` |

Limits: a node now has up to 32 RPDOs (was 4), one per remote node TPDO, 8 mapping entries each.

## Not available yet
* `template_entities` (custom OD entries without an ESPHome entity).
* `canopen.entity_cmd` (a raw command to a remote entity). For other devices, the `canopen` component writes objects
  directly: the `canopen.sdo_write` action, `write_sdo()` in lambdas.
* `csdo:` and its lambdas (`setup_csdo`, `csdo_recv`, `csdo_send_*`): use the `canopen` component's SDO client
  (`read_sdo()`, `write_sdo()`, `read_sdo_data()`, `write_sdo_data()`).

## Stored settings
PDO settings a tool changed over SDO and stored with `0x1010` on the old component (ESP32 only) aren't carried over:
the new node starts with the PDOs from its YAML.

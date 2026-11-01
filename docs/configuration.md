# Configuration
Reference of the `canopen_entities:` component. For a first setup see the [Quick start](../README.md#quick-start).

A node has two parts: the CANopen node itself (`canopen:`, the hub: node id, heartbeat, SDO, PDOs), and
`canopen_entities:` on top of it, which exposes ESPHome entities in the node's object dictionary and sends their
states in TPDOs.

```yaml
canopen:
  node_id: 0x10            # unique on the bus, 1..127
  heartbeat_interval: 5s

canopen_entities:
  entities:                # ESPHome entities exposed over CAN
    - id: relay1           # id of a switch defined elsewhere in the YAML
      index: 1             # stable entity index 1..255, don't renumber later
      tpdo: 0              # broadcast state changes in TPDO 0
    - id: temperature
      index: 2
      tpdo: 0
      size: 2              # send as scaled UINT16 instead of REAL32...
      min_value: -40       # ...over this range
      max_value: 85
```

The TPDO a state is sent in, and the byte offset inside it, are computed from this config. The build logs them for
each node ([TPDO layout](../OBJECT_DICTIONARY.md#tpdo-layout)).

## `canopen:` options
The `canopen` component is documented on its own
[page](https://github.com/mrk-its/esphome-docs/blob/canopen-full/src/content/docs/components/canopen.mdx) (a draft of
its esphome.io page). The ones that matter for an entity node:
* `node_id` (Required, int, 1..127): the node id, unique on the bus.
* `canbus_id` (Optional, id): the `canbus` to use, needed only with more than one bus.
* `heartbeat_interval` (Optional, time, default=5s).
* `heartbeat_clients`, `on_node_event`: monitor the heartbeats of other nodes (online / offline / NMT state / reset).
* `on_nmt_state_change`, `nmt_startup`: this node's own NMT state.
* `sdo_block_size` (Optional, int, 1..127): frames per block of an SDO block download. `canopen_entities` needs block
  transfer for OTA and remote logs; without this option it uses 16. A block arrives back to back, so it must fit the
  canbus RX queue (e.g. `zephyr_can` `rx_buffer_size`).
* `hardware_version`, `device_name` (Optional, string): `0x1009` / `0x1008`. can2mqtt names the device after
  `device_name`, which defaults to the node's name.

`canopen_entities` sets the identity (`0x1018`, the ESPHome values can2mqtt looks for) and the software version
(`0x100A`, a version string with the build time) itself; `identity` and `software_version` on `canopen:` are
overwritten.

## `canopen_entities:` options
One `canopen_entities:` per firmware.
* `canopen_id` (Optional, id): the `canopen` node, defaults to the only one.
* `entities` (Optional, list of `entity` objects): ESPHome entities exposed via CANopen, see the `entity` schema below.
* `od_writer` (Optional, bool, default=true): node-to-node commands (the `canopen_entities.*` actions of other
  nodes, see [OD writer](../OBJECT_DICTIONARY.md#od-writer)). Reserves TPDO 7.
* `remote_nodes` (Optional, mapping of name to YAML file): other nodes this node receives states from or sends commands
  to, by the ESPHome YAML file they are built from (path relative to this config). Their TPDO layout is computed from
  that file, so `rpdo`, `platform: canopen_entities` entities and the `canopen_entities.*` actions can refer to remote
  entities by id. See [Linking nodes](linking-nodes.md).
* `remote_logs` (Optional): send the node's log output over CAN to can2mqtt, see
  [Remote logging over CAN](remote-logs.md).

## `entity` schema
* `id` (Required, id): id of the ESPHome entity.
* `index` (Required, int, 1..255): index of the entity. Together with `node_id` it is the unique id of the
  CAN-exposed entity in Home Assistant, so change it with care. Indices must be unique.
* `tpdo` (Optional, `TPDO` schema, see below): broadcast state changes in this TPDO.
* `rpdo` (Optional, list of `RPDO` schema, see below): states of other nodes, received from their TPDOs and written
  into this entity's commands.
* `size` (Optional, int, `sensor` / `number` only): size of the state on the wire: `4` (default, REAL32), `1` (scaled
  UINT8) or `2` (scaled UINT16). See [Sensor8 / Sensor16](../OBJECT_DICTIONARY.md#sensor8--sensor16).
* `min_value` (Optional, float, default=0): minimal value for scaled states (`size` 1 or 2).
* `max_value` (Optional, float, default=254 for `size: 1`, 65534 for `size: 2`): maximal value for scaled states
  (`size` 1 or 2).
* `caps` (Optional, list, `light` / `cover` only): capabilities of the entity, needed only for platforms whose
  capabilities the component can't derive from the config (the build fails with a hint). Light: `on_off`,
  `brightness`, `color_temperature`; cover: `position`, `tilt`. See [TPDO layout](../OBJECT_DICTIONARY.md#tpdo-layout).

## `TPDO` schema
Any of:
- an integer: the TPDO number, 0..7 (7 is reserved for the OD writer unless `od_writer: false`);
- an object:
  * `number` (Required, int, 0..7): the TPDO number.
  * `immediate` (Optional, bool): accepted for configs of the old component, no effect: every change is sent at the
    next CANopen processing (several changes in one loop go out as one frame).

TPDOs are sent on change; PDOs are only sent in the NMT state OPERATIONAL (the default after boot).

## `RPDO` schema
Any of:
- remote entity of a node from `remote_nodes`:
  * `node` (Required, string): name of the node in `remote_nodes`.
  * `entity` (string) or `index` (int) (one of them Required): id or index of the remote entity (must be in the remote
    node's `entities`, with `tpdo`).
  * `state` (Optional, string): remote entity state, defaults to the first one: `state` (light, switch, binary sensor,
    cover, alarm) or `value` (sensor, number). Others: light `brightness`, `color_temp`; cover `position`, `tilt`.
  * `cmd` (Optional, int, default=0): entity command index the state is mapped to; its size must match the remote
    state's size.
- raw TPDO position (for nodes not described by an ESPHome YAML):
  * `node_id` (Required, int, 1..127): id of the node sending the TPDO.
  * `tpdo` (Required, int, 0..7): its TPDO number.
  * `offset` (Required, int, 0..7): byte offset inside the TPDO frame. One byte is mapped.
  * `cmd` (Optional, int, default=0): entity command index (starting from 0) the byte is written to; the command must
    be 1 byte.
  * `tpdo_size` (Optional, int, 1..8): the length of that TPDO. Without it the RPDO ends at the last mapped byte: a
    longer frame is still processed, but the node reports it once with an emergency message (EMCY).

A node has one RPDO per remote node TPDO it reads (up to 32), each with at most 8 mapping entries (gaps included),
shared with the [remote states](linking-nodes.md#remote-states).

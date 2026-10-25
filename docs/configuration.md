# Configuration
Reference of the `canopen:` component. For a first setup see the [Quick start](../README.md#quick-start).

```yaml
canopen:
  node_id: 0x10            # unique on the bus, 1..127
  entities:                # ESPHome entities exposed over CAN
    - id: relay1           # id of a switch defined elsewhere in the YAML
      index: 1             # stable entity index 1..64, don't renumber later
      tpdo: 0              # broadcast state changes in TPDO 0
    - id: temperature
      index: 2
      tpdo: {number: 0, immediate: true}
      size: 2              # send as scaled UINT16 instead of REAL32...
      min_value: -40       # ...over this range
      max_value: 85
  heartbeat_interval: 5s
```

The TPDO a state is sent in, and the byte offset inside it, are computed from this config. The build logs them for
each node ([TPDO layout](../OBJECT_DICTIONARY.md#tpdo-layout)).

## `canopen:` options
* `node_id` (Required, int): canopen node_id, in 0..127 range
* `canbus_id` (Optional, id): id of `canbus` component to use, defaults to the only configured one
* `entities` (Required, list of `entity` objects): list of ESPHome entities exposed via CANopen, see `entity` schema below
* `heartbeat_interval` (Optional, time interval, default=5000ms): Heartbeat interval
* `heartbeat_clients` (Optional, list of `heartbeat_client`): list of nodes to track heartbeat messages for, see below.
* `on_pre_operational` (Optional, Automation): An automation to perform when node enters pre_operational state
* `on_operational` (Optional, Automation): An automation to perform when node enters operational state
* `on_hb_consumer_event` (Optional, Automation): An automation to perform when heartbeat clients are configured and heartbeat event occurs. `node_id` (uint8) variable is available in the automation.
* `sdo_block_transfer_size` (Optional, int, 1..127, default=63): number of messages confirmed with single ACK for SDO block transfer mode. This is the SDO buffer (7 bytes of RAM per message): the block size of SDO downloads (CAN OTA, should fit the canbus RX queue) and the largest upload block (`remote_logs: block_size`)
* `pdo_od_writer` (Optional, bool, default=True): when enabled then `RPDO #3` is reserved for node to node communication (remote OD writes, see [OD writer](../OBJECT_DICTIONARY.md#od-writer))
* `hw_version` (Optional, string): value of Manufacturer Hardware Version (`0x1009`)
* `sw_version` (Optional, string): currently ignored, `0x100A` always contains generated ESPHome version string (see [#30](https://github.com/mrk-its/esphome-canopen/issues/30))
* `template_entities` (Optional, list of `template_entity` objects): custom OD entities not backed by ESPHome entity, see below
* `remote_logs` (Optional): send the node's log output over CAN to can2mqtt, see [Remote logging over CAN](remote-logs.md)
* `remote_nodes` (Optional, mapping of name to YAML file): other nodes this node receives states from or sends commands to, by the ESPHome YAML file they are built from (path relative to this config). Their TPDO layout is computed from that file, so `rpdo`, `platform: canopen` entities and the `canopen.*` actions can refer to remote entities by id. See [Linking nodes](linking-nodes.md)

## `entity` schema
* `id` (Required, string): id of ESPHome entity
* `index` (Required, int): index of entity, range 1..64. Together with `node_id` forms unique id of CAN-exposed entity, so it should be changed with a care. Indices must be unique.
* `tpdo` (Optional, `TPDO` schema (see below)): when defined then state changes will be broadcasted via TPDO
* `rpdo` (Optional, list of `RPDO` schema (see below)): when defined then received TPDO frames will be automatically mapped to OD entity command entries
* `size` (Optional, int, `sensor` / `number` only): size of state on the wire: `4` (default, REAL32), `1` (scaled UINT8) or `2` (scaled UINT16). See [Sensor8 / Sensor16](../OBJECT_DICTIONARY.md#sensor8--sensor16)
* `min_value` (Optional, float, default=0): minimal value for scaled states (`size` 1 or 2)
* `max_value` (Optional, float, default=254 for `size: 1`, 65534 for `size: 2`): maximal value for scaled states (`size` 1 or 2)
* `caps` (Optional, list, `light` / `cover` only): capabilities of the entity, needed only for platforms whose capabilities the component can't derive from the config (the build fails with a hint). Light: `on_off`, `brightness`, `color_temperature`; cover: `position`, `tilt`. See [TPDO layout](../OBJECT_DICTIONARY.md#tpdo-layout)

## `TPDO` schema
Any of:
- object with following properties:
  * `number` (Required, integer): integer in 0..7 range (7 is typically reserved for node to node communication, unless `pdo_od_writer` is disabled, see above)
  * `immediate` (Optional, bool, default=false): When true then TPDO is sent immediately on every state change. When false, changed TPDOs are marked dirty and sent once per component loop (several changes in one loop go out as one frame).
- integer representing `number` defined above (equivalent to `{number: N}`, `immediate: false`).

## `RPDO` schema
Any of:
- remote entity of a node from `remote_nodes`:
  * `node` (Required, string): name of the node in `remote_nodes`
  * `entity` (string) or `index` (int) (one of them Required): id or index of the remote entity (must be in the remote node's `canopen.entities`, with `tpdo`)
  * `state` (Optional, string): remote entity state, defaults to the first one: `state` (light, switch, binary sensor, cover, alarm) or `value` (sensor, number). Others: light `brightness`, `color_temp`; cover `position`, `tilt`
  * `cmd` (Optional, defaults to 0): entity command index the state is mapped to; its size must match the remote state's size
- raw TPDO position:
  * `node_id` (Required, integer): id of node sending mapped TPDO frame
  * `tpdo` (Required, integer): TPDO number
  * `offset` (Required, integer): byte offset inside TPDO frame, 0..7 range
  * `cmd` (Optional, defaults to 0): entity command index (starting from 0) where received TPDO fragment is mapped to. One byte is mapped.

A node has at most 4 RPDOs (one per remote node TPDO), each with at most 8 mapping entries (gaps included).

## `heartbeat_client` schema
* `node_id` (Required, int): tracked node id
* `timeout` (Required, time interval): when exceeded `on_hb_consumer_event` automations will be triggered

## `template_entity` schema
Template entities allow to define custom OD entries at given entity index, without ESPHome entity behind them.
* `index` (Required, int): entity index (same meaning as in `entity`)
* `tpdo` (Optional, int, default=-1): TPDO number states are mapped to
* `metadata` (Optional): entity metadata
  * `type` (Required, int): EntityTypeCode
  * `name`, `device_class`, `unit`, `state_class` (Optional, string)
* `states` (Optional, list): each item has `type` (Required, one of `uint8`, `uint16`, `uint32`, `int8`, `int16`, `int32`)
* `commands` (Optional, list): each item has:
  * `type` (Required, one of `uint8`, `uint16`, `uint32`)
  * `handler` (Required, Automation): automation run when command is written, value is available as `x` variable

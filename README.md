# ESPHome CANopen external component

This project provides ESPHome external_component named `canopen` for easy conversion of ESPHome entities into CANopen nodes. It allows to [map selected entities](OBJECT_DICTIONARY.md) to CANopen object dictionary. It also allows to define TPDOs for sending entity state on change. Currently following entity types are supported: `sensor`, `number`, `binary_sensor`, `switch`, `cover`, `light` and `alarm_control_panel`. Entity metadata are also published in Object Dictionary enabling autodiscovery.

The [can2mqtt](https://github.com/mrk-its/can2mqtt) project provides can2mqtt bridge exposing ESPHome CANopen entities onto MQTT topics. It follows MQTT Discovery protocol, so entities appear automatically in HomeAssistant.

## Configuration
`canopen` platform recognizes following parameters:
* `node_id` (Required, int): canopen node_id, in 0..127 range
* `canbus_id` (Optional, id): id of `canbus` component to use, defaults to the only configured one
* `entities` (Required, list of `entity` objects): list of ESPHome entities exposed via CANopen, see `entity` schema below
* `heartbeat_interval` (Optional, time interval, default=5000ms): Heartbeat interval
* `heartbeat_clients` (Optional, list of `heartbeat_client`): list of nodes to track heartbeat messages for, see below.
* `on_pre_operational` (Optional, Automation): An automation to perform when node enters pre_operational state
* `on_operational` (Optional, Automation): An automation to perform when node enters operational state
* `on_hb_consumer_event` (Optional, Automation): An automation to perform when heartbeat clients are configured and heartbeat event occurs. `node_id` (uint8) variable is available in the automation.
* `sdo_block_transfer_size` (Optional, int, 1..127, default=63): number of messages confirmed with single ACK for SDO block transfer mode
* `pdo_od_writer` (Optional, bool, default=True): when enabled then `RPDO #3` is reserved for node to node communication (remote OD writes, see [OD writer](OBJECT_DICTIONARY.md#od-writer))
* `hw_version` (Optional, string): value of Manufacturer Hardware Version (`0x1009`)
* `sw_version` (Optional, string): currently ignored, `0x100A` always contains generated ESPHome version string (see [#30](https://github.com/mrk-its/esphome-canopen/issues/30))
* `template_entities` (Optional, list of `template_entity` objects): custom OD entities not backed by ESPHome entity, see below
* `remote_nodes` (Optional, mapping of name to YAML file): other nodes this node receives states from or sends commands to, by the ESPHome YAML file they are built from (path relative to this config). Their TPDO layout is computed from that file, so `rpdo` and the `canopen.*` actions can refer to remote entities by id. See [Linking nodes](#linking-nodes)

### `entity` schema:
* `id` (Required, string): id of ESPHome entity
* `index` (Required, int): index of entity, range 1..64. Together with `node_id` forms unique id of CAN-exposed entity, so it should be changed with a care. Indices must be unique.
* `tpdo` (Optional, `TPDO` schema (see below)): when defined then state changes will be broadcasted via TPDO
* `rpdo` (Optional, list of `RPDO` schema (see below)): when defined then received TPDO frames will be automatically mapped to OD entity command entries
* `size` (Optional, int, `sensor` / `number` only): size of state on the wire: `4` (default, REAL32), `1` (scaled UINT8) or `2` (scaled UINT16). See [Sensor8 / Sensor16](OBJECT_DICTIONARY.md#sensor8--sensor16)
* `min_value` (Optional, float, default=0): minimal value for scaled states (`size` 1 or 2)
* `max_value` (Optional, float, default=254 for `size: 1`, 65534 for `size: 2`): maximal value for scaled states (`size` 1 or 2)
* `caps` (Optional, list, `light` / `cover` only): capabilities of the entity, needed only for platforms whose capabilities the component can't derive from the config (the build fails with a hint). Light: `on_off`, `brightness`, `color_temperature`; cover: `position`, `tilt`. See [TPDO layout](OBJECT_DICTIONARY.md#tpdo-layout)

### `TPDO` schema:
Any of:
- object with following properties:
  * `number` (Required, integer): integer in 0..7 range (7 is typically reserved for node to node communication, unless `pdo_od_writer` is disabled, see above)
  * `is_async` (Optional, bool, default=true): When true then TPDO is sent immediately on every state change. When false, changed TPDOs are marked dirty and sent once per component loop.
- integer representing `number` defined above. Note: in this form `is_async` is **false**.

### `RPDO` schema:
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

### `heartbeat_client` schema:
* `node_id` (Required, int): tracked node id
* `timeout` (Required, time interval): when exceeded `on_hb_consumer_event` automations will be triggered

### `template_entity` schema:
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

## Linking nodes
A node can mirror states of another node's entities (via its TPDOs) and send commands to them. With
`remote_nodes` the other node is referred to by the YAML file it's built from, and its entities by id:
TPDO numbers, offsets, node ids and entity indices are taken from that file. E.g. a wall switch node
controlling a light of a dimmer node:

```yaml
# dimmer3.yaml
canopen:
  node_id: 0x36
  entities:
    - id: light_mudroom
      index: 3
      tpdo: 1
```

```yaml
# wall-switch.yaml
canopen:
  node_id: 0x62
  remote_nodes:
    dimmer3: dimmer3.yaml
  entities:
    - id: light_state_mudroom        # local mirror of the remote light state
      index: 2
      rpdo: {node: dimmer3, entity: light_mudroom}
    - id: brightness_mudroom         # optional: remote brightness, 0..1
      index: 3
      size: 1
      min_value: 0
      max_value: 1
      rpdo: {node: dimmer3, entity: light_mudroom, state: brightness}

binary_sensor:
  - platform: template
    id: light_state_mudroom
  - platform: gpio
    pin: ...
    on_press:
      - canopen.light_toggle: {node: dimmer3, entity: light_mudroom}

sensor:
  - platform: template
    id: brightness_mudroom
```

Only the remote YAML's `canopen:` section and the configs of its exposed entities are read (after
packages and substitutions); the remote node isn't built. Rebuild the linked nodes after changing the
TPDO layout of a remote node (the build logs each node's [TPDO layout](OBJECT_DICTIONARY.md#tpdo-layout)).

### Actions
* `canopen.entity_cmd`: writes a command of a remote entity (via [OD writer](OBJECT_DICTIONARY.md#od-writer)):
  * `node` + `entity` / `index` (remote node from `remote_nodes`), or `node_id` + `index`
  * `cmd` (Optional, int, default=0): command index, see entity types in [OBJECT_DICTIONARY.md](OBJECT_DICTIONARY.md)
  * `value` (Required, templatable uint8): command value
* `canopen.light_toggle`: toggles a remote light / switch: sends the inverse of its local mirror's state as command 0.
  * `node` + `entity` / `index`: remote entity (from `remote_nodes`)
  * `state_id` (Optional, id): local binary sensor / switch mirroring the remote state; by default the local entity
    with `rpdo` of that remote entity's state.

  The mirror is only up to date after the remote node has sent its TPDO (e.g. after a reboot of this node the first
  press may switch the light on even if it's already on), see [#32](https://github.com/mrk-its/esphome-canopen/issues/32).

## OTA over CAN
The `canopen` OTA platform allows to flash firmware over CAN bus:
```
ota:
  - platform: canopen
```

### Zephyr (`platform: zephyr`) with MCUboot
On `platform: zephyr` the update is written to MCUboot's secondary slot and swapped in on the next reboot:
* MCUboot is required: set `advanced: bootloader: mcuboot` in the `zephyr:` block (with a partition layout that has `slot0_partition` / `slot1_partition`).
* `swap_method:` (`scratch`, `move` or `offset`) can be set on the `canopen` OTA platform, same as on `platform: esphome`.
* Upload the signed image: `.esphome/build/<node>/.west_build/zephyr/zephyr/zephyr.signed.bin`. Never upload `zephyr.bin` (unsigned); the node rejects it at the start of the transfer.
* The new image boots in test mode. `safe_mode` confirms it after a good boot; if it's never confirmed, MCUboot rolls back to the previous image on the next reboot.
* OTA compression is disabled on Zephyr (and on stm32), so can2mqtt sends the image uncompressed.

# How to start
## Hardware requirements
 * any ESP8266/ESP32 board with CAN controller. ESP32 boards are preferred, as they have [integrated can controller](https://esphome.io/components/canbus.html#esp32-can-component) (but still cheap external CAN transceiver is needed). For ESP8266 external [MCP2515](https://esphome.io/components/canbus.html#mcp2515-component) CAN controller can be used.
## Software requirements
 * [ESPHome](https://esphome.io/) installed

## Configure ESPHome with `canopen` external_component

  Example configuration for esp32dev board:

  ```
  esphome:
    name: can-node-1

  external_components:
    - source: github://mrk-its/esphome-canopen

  # Enable logging
  logger:

  # Enable Home Assistant API
  api:
    reboot_timeout: 0s

  ota:
    - platform: esphome
    - platform: canopen

  esp32:
    board: esp32dev
    framework:
      type: esp-idf

  wifi:
    <<: !include {file: wifi.yaml}

  canbus:
    - platform: esp32_can
      id: can_bus
      rx_pin: GPIO22
      tx_pin: GPIO23
      can_id: 0
      bit_rate: 125kbps

  canopen:
    id: can_gate
    canbus_id: can_bus
    node_id: 1
    entities:
      - id: boot
        index: 1
        tpdo: 0
      - id: blue_led
        index: 2
        tpdo: 0
      - id: uptime_sensor
        index: 3
        tpdo: 0
      - id: cover1
        index: 4
        tpdo: 1

  sensor:
    - platform: uptime
      id: uptime_sensor
      name: "Uptime 1"
      update_interval: 5sec
      internal: true

  binary_sensor:
    - platform: gpio
      name: "Boot 1"
      id: boot
      internal: true
      pin:
        number: 0
        inverted: true

  switch:
    - platform: gpio
      name: "Led 1"
      id: blue_led
      internal: true
      pin: 2

  cover:
    - platform: time_based
      name: "Cover 1"
      id: cover1
      internal: true
      device_class: shutter
      has_built_in_endstop: true
      open_action:
        - logger.log: open_action
      open_duration: 10s
      close_action:
        - logger.log: close_action
      close_duration: 10s
      stop_action:
        - logger.log: stop_action

  ```

More examples can be found in [examples](examples/) directory.

# Support
## Community

* [Discord Server](https://discord.gg/VXjUSnUWsd)

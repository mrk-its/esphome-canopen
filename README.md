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

### `entity` schema:
* `id` (Required, string): id of ESPHome entity
* `index` (Required, int): index of entity, range 1..64. Together with `node_id` forms unique id of CAN-exposed entity, so it should be changed with a care. Indices must be unique.
* `tpdo` (Optional, `TPDO` schema (see below)): when defined then state changes will be broadcasted via TPDO
* `rpdo` (Optional, list of `RPDO` schema (see below)): when defined then received TPDO frames will be automatically mapped to OD entity command entries
* `size` (Optional, int, `sensor` / `number` only): size of state on the wire: `4` (default, REAL32), `1` (scaled UINT8) or `2` (scaled UINT16). See [Sensor8 / Sensor16](OBJECT_DICTIONARY.md#sensor8--sensor16)
* `min_value` (Optional, float, default=0): minimal value for scaled states (`size` 1 or 2)
* `max_value` (Optional, float, default=254 for `size: 1`, 65534 for `size: 2`): maximal value for scaled states (`size` 1 or 2)

### `TPDO` schema:
Any of:
- object with following properties:
  * `number` (Required, integer): integer in 0..7 range (7 is typically reserved for node to node communication, unless `pdo_od_writer` is disabled, see above)
  * `is_async` (Optional, bool, default=true): When true then TPDO is sent immediately on every state change. When false, changed TPDOs are marked dirty and sent once per component loop.
- integer representing `number` defined above. Note: in this form `is_async` is **false**.

### `RPDO` schema:
* `node_id` (Required, integer): id of node sending mapped TPDO frame
* `tpdo` (Required, integer): TPDO number
* `offset` (Required, integer): byte offset inside TPDO frame, 0..7 range
* `cmd` (Optional, defaults to 0): entity command index (starting from 0) where received TPDO fragment is mapped to. It also determines size of mapped fragment.

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

## OTA over CAN
The `canopen` OTA platform allows to flash firmware over CAN bus:
```
ota:
  - platform: canopen
```

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

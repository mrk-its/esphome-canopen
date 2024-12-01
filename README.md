# ESPHome CANopen external component

This project provides ESPHome external_component named `canopen` for easy conversion of ESPHome entities into CANopen nodes. It allows to [map selected entities](OBJECT_DICTIONARY.md) to CANopen object dictionary. It also allows to define TPDOs for sending entity state on change. Currently following entity types are supported: `sensor`, `number`, `binary_sensor`, `switch`, `cover`, `light` and `alarm_control_panel`. Entity metadata are also published in Object Dictionary enabling autodiscovery.

The [can2mqtt](https://github.com/mrk-its/can2mqtt) project provides can2mqtt bridge exposing ESPHome CANopen entities onto MQTT topics. It follows MQTT Discovery protocol, so entities appear automatically in HomeAssistant.

## Configuration
`canopen` plafrom recognizes following parameters:
* `node_id` (Required, int): canopen node_id, in 0..127 range
* `heartbeat_interval` (Optional, time interval, default=5000ms): Heartbeat interval
* `on_pre_operational` (Optional, Automation): An automation to perform when node enters pre_operational state
* `on_operational` (Optional, Automation): An automation to perform when node enters  perational state
* `on_hb_consumer_event` (Optional, Automation): An automation to perform when heartbeat clients are configured and heartbeat is received
* `sdo_block_transfer_size` (Optional, int, defaults to 63): number of messages confirmed with single ACK for SDO block transfer mode
* `heartbeat_clients` (Optional, list of 'heartbeat_client'): list of nodes to track hearbeat messages for, see below.

* `pdo_od_writer` (Optional, bool, default=True): when enabled then `RPDO #3` is reserved for node to node communication (remote OD writes)
* `entities` (Optional, list of `entity` objects): list of ESPHome entities exposed via CANOpen, see `entity` schema below

### `entity` schema:
* `id` (Required, string): id of ESPHome entity
* `index` (Required, int): index of entity, range 1..64. Together with `node_id` forms unique id of CAN-exposed entity, so it should be changed with a care.
* `tpdo` (Optional, `TPDO` schema (see below)): when defined then state changes will be broadcasted via TPDO
* `rpdo` (Optional, `RPDO` schema (see below)): when defined then received TPDO frames will be automatically mapped to OD entity command entries

### `TPDO` schema:
Any of:
- object with following properties:
  * `number` (Required, integer): integer in 0..7 range (7 is typically reserved for node to node communication, unless `pdo_od_writer` is disabled, see above)
  * `is_async` (Optinal, bool, default=true): When true then state is automaticall published on change. When false, TPDO transmission needs to be manually triggered
- integer representing `number` defined above.

### `RPDO` schema:
* `node_id` (Required, integer): id of node sending mapped TPDO frame
* `tpdo` (Required, integer): TPDO number
* `offset` (Required, integer): TPDO offset, 0..7 range
* `cmd` (Optional, defaults to 0): current entity command index (starting from 0) where received TPDO fragemnt is mapped to, it also determines size of mappend fragment.

### `heartbeat_client` schema:
* `node_id` (Required, int): tracked node id
* `timeout` (Required, time interval): when exceeded `on_hb_consumer_event` automations will be triggered




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
    password: ""
    reboot_timeout: 0s

  ota:
    password: ""

  esp32:
    board: esp32dev
    framework:
      type: arduino

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
      - id: cover2
        index: 5
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

# Support
## Community

* [Discord Server](https://discord.gg/VXjUSnUWsd)

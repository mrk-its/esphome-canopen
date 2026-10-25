# Linking nodes
A node can read states of another node's entities (from its TPDOs) and send commands to them. With
`remote_nodes` the other node is referred to by the YAML file it's built from, and its entities by id:
TPDO numbers, offsets, node ids and entity indices are taken from that file. E.g. a wall switch node
controlling lights of a dimmer node:

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

binary_sensor:
  - platform: gpio
    pin: ...
    on_press:
      - canopen.light.toggle: {node: dimmer3, entity: light_mudroom}

sensor:
  - platform: canopen              # optional: remote state as a local entity
    id: mudroom_brightness
    node: dimmer3
    entity: light_mudroom
    state: brightness
```

Only the remote YAML's `canopen:` section and the configs of its exposed entities are read (after
packages and substitutions); the remote node isn't built. Rebuild the linked nodes after changing the
TPDO layout of a remote node (the build logs each node's [TPDO layout](../OBJECT_DICTIONARY.md#tpdo-layout)).

## Remote states
Remote states a node needs (by `platform: canopen` entities and toggle actions) are mirrored in
[remote state slots](../OBJECT_DICTIONARY.md#remote-state-slots), one per remote state, filled by RPDOs from the
remote node's TPDOs. They don't show up in Home Assistant. The RPDO limits apply (4 RPDOs, one per remote node
TPDO, 8 mapping entries each, together with the `rpdo` entries of `entities`); a remote state can't be mapped by
both a slot and an entity's `rpdo`.

The remote state is only known after the remote node has sent its TPDO, e.g. after a reboot of this node a toggle
may switch a light on even if it's already on, see [#32](https://github.com/mrk-its/esphome-canopen/issues/32).

## `platform: canopen` entities
`binary_sensor`, `sensor` and `text_sensor` showing a remote state:
* `node` (Required, string): name of the node in `remote_nodes`
* `entity` (Required, string): id of the remote entity (in the remote node's `canopen.entities`, with `tpdo`)
* `state` (Optional, string): remote entity state, defaults to the first one: `state` (light, switch, binary sensor,
  cover, alarm) or `value` (sensor, number). Others: light `brightness`, `color_temp`; cover `position`, `tilt`
* all options of the entity type (`name`, `filters`, `on_value`, ...). Without `name` the entity is internal.

| Platform | Remote states | Value |
|---|---|---|
| `binary_sensor` | light / switch / binary sensor `state` | |
| `sensor` | any | `state`: 0 / 1; brightness, position, tilt: 0..1; `color_temp`: mireds; cover / alarm `state`: code (see [OBJECT_DICTIONARY.md](../OBJECT_DICTIONARY.md)); sensor / number `value` |
| `text_sensor` | light / switch / binary sensor `state`, cover `state`, alarm `state` | `on` / `off`; `open`, `opening`, `closed`, `closing`; `disarmed`, `armed_home`, ... |

A `platform: canopen` entity can be listed in `canopen.entities` (without `rpdo`), e.g. to see the remote state
through can2mqtt while debugging.

## Actions
All take `node` (name in `remote_nodes`) and `entity` (id) or `index` of the remote entity; the remote entity's type
is checked against the action. Commands are sent with the [OD writer](../OBJECT_DICTIONARY.md#od-writer). Toggles
read the remote state from a remote state slot, so the remote entity needs `tpdo`.

| Action | Remote entity | Options |
|---|---|---|
| `canopen.light.turn_on` | light, switch | `brightness` (percentage), `color_temperature`, both templatable |
| `canopen.light.turn_off`, `canopen.light.toggle` | light, switch | |
| `canopen.switch.turn_on`, `.turn_off`, `.toggle` | switch, light | |
| `canopen.cover.open`, `.close`, `.stop` | cover | |
| `canopen.cover.toggle` | cover | stops a moving cover, opens a closed one, otherwise closes |
| `canopen.cover.control` | cover | `stop` (bool), `position`, `tilt` (percentage, templatable) |
| `canopen.alarm_control_panel.disarm`, `.arm_away`, `.arm_home`, `.arm_night`, `.arm_vacation`, `.arm_custom_bypass` | alarm control panel | |
| `canopen.number.set` | number | `value` (Required, templatable float), scaled as the remote number |

`canopen.entity_cmd` writes a raw command of a remote entity:
* `node` + `entity` / `index` (remote node from `remote_nodes`), or `node_id` + `index`
* `cmd` (Optional, int, default=0): command index, see entity types in [OBJECT_DICTIONARY.md](../OBJECT_DICTIONARY.md)
* `value` (Required, templatable uint8): command value

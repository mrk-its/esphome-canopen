# Linking nodes
A node can read states of another node's entities (from its TPDOs) and send commands to them. With
`remote_nodes` the other node is referred to by the YAML file it's built from, and its entities by id:
TPDO numbers, offsets, node ids and entity indices are taken from that file. E.g. a wall switch node
controlling lights of a dimmer node:

```yaml
# dimmer3.yaml
canopen:
  node_id: 0x36

canopen_entities:
  entities:
    - id: light_mudroom
      index: 3
      tpdo: 1
```

```yaml
# wall-switch.yaml
canopen:
  node_id: 0x62

canopen_entities:
  remote_nodes:
    dimmer3: dimmer3.yaml

binary_sensor:
  - platform: gpio
    pin: ...
    on_press:
      - canopen_entities.light.toggle: {node: dimmer3, entity: light_mudroom}

sensor:
  - platform: canopen_entities     # optional: remote state as a local entity
    id: mudroom_brightness
    node: dimmer3
    entity: light_mudroom
    state: brightness
```

Only the remote YAML's `canopen:` / `canopen_entities:` sections and the configs of its exposed entities are read
(after packages and substitutions); the remote node isn't built. A remote node may also still run the old component
(its entities in `canopen: entities:`): both formats are read. Rebuild the linked nodes after changing the TPDO
layout of a remote node (the build logs each node's [TPDO layout](../OBJECT_DICTIONARY.md#tpdo-layout)).

## Remote states
Remote states a node needs (by `platform: canopen_entities` entities and toggle actions) are mirrored in
[remote state slots](../OBJECT_DICTIONARY.md#remote-state-slots), one per remote state, filled by RPDOs from the
remote node's TPDOs. They don't show up in Home Assistant. The node has one RPDO per remote node TPDO (up to 32),
each with at most 8 mapping entries, shared with the `rpdo` entries of `entities`; a remote state can't be mapped by
both a slot and an entity's `rpdo`.

The remote state is only known after the remote node has sent its TPDO. A node therefore asks the remote nodes it
listens to for the TPDOs it needs ([TPDO request](../OBJECT_DICTIONARY.md#tpdo-request)): 50..500 ms after it becomes
OPERATIONAL, and again when one of them boots. Until the answer arrives, or if the remote node's firmware is too old to
answer (the old component isn't asked at all), a toggle may switch a light on even if it's already on. The remote node
also answers every command it gets through the OD writer with its state, even if nothing changed, so the second press
works there.

A TPDO goes to everyone listening. A request therefore also reaches other nodes with an `rpdo` entry on the same TPDO,
which run it as a command again, like the remote node's own TPDO at its boot.

## `platform: canopen_entities` entities
`binary_sensor`, `sensor` and `text_sensor` showing a remote state:
* `node` (Required, string): name of the node in `remote_nodes`
* `entity` (Required, string): id of the remote entity (in the remote node's `entities`, with `tpdo`)
* `state` (Optional, string): remote entity state, defaults to the first one: `state` (light, switch, binary sensor,
  cover, alarm) or `value` (sensor, number). Others: light `brightness`, `color_temp`; cover `position`, `tilt`
* all options of the entity type (`name`, `filters`, `on_value`, ...). Without `name` the entity is internal.

| Platform | Remote states | Value |
|---|---|---|
| `binary_sensor` | light / switch / binary sensor `state` | |
| `sensor` | any | `state`: 0 / 1; brightness, position, tilt: 0..1; `color_temp`: mireds; cover / alarm `state`: code (see [OBJECT_DICTIONARY.md](../OBJECT_DICTIONARY.md)); sensor / number `value` |
| `text_sensor` | light / switch / binary sensor `state`, cover `state`, alarm `state` | `on` / `off`; `open`, `opening`, `closed`, `closing`; `disarmed`, `armed_home`, ... |

A `platform: canopen_entities` entity can be listed in `canopen_entities: entities:` (without `rpdo`), e.g. to see the
remote state through can2mqtt while debugging.

## Actions
All take `node` (name in `remote_nodes`) and `entity` (id) or `index` of the remote entity; the remote entity's type
is checked against the action. Commands are sent with the [OD writer](../OBJECT_DICTIONARY.md#od-writer), so the
remote node must have it on (`od_writer`, the default); `od_writer_mpdo` on the sending node selects the frame format
([options](configuration.md)). Toggles read the remote state from a remote state slot, so
the remote entity needs `tpdo`.

| Action | Remote entity | Options |
|---|---|---|
| `canopen_entities.light.turn_on` | light, switch | `brightness` (percentage), `color_temperature`, both templatable |
| `canopen_entities.light.turn_off`, `canopen_entities.light.toggle` | light, switch | |
| `canopen_entities.switch.turn_on`, `.turn_off`, `.toggle` | switch, light | |
| `canopen_entities.cover.open`, `.close`, `.stop` | cover | |
| `canopen_entities.cover.toggle` | cover | stops a moving cover, opens a closed one, otherwise closes |
| `canopen_entities.cover.control` | cover | `stop` (bool), `position`, `tilt` (percentage, templatable) |
| `canopen_entities.alarm_control_panel.disarm`, `.arm_away`, `.arm_home`, `.arm_night`, `.arm_vacation`, `.arm_custom_bypass` | alarm control panel | |
| `canopen_entities.number.set` | number | `value` (Required, templatable float), scaled as the remote number |

The old component's `canopen.entity_cmd` (a raw command to a remote entity) isn't available yet. For other devices
on the bus, the `canopen` component's actions and lambdas write objects directly (`canopen.sdo_write`, `write_sdo()`).

## Wall buttons (`canopen_buttons`)
The `canopen_buttons` component (in this repository, next to `canopen_entities`, which it needs) turns buttons into
wall controls for entities of remote nodes. Nothing is needed on the remote node beyond the entity's usual commands
(the OD writer, `tpdo` on the entity). With a `components:` filter on the `external_components` entry, add
`canopen_buttons` to it.

### Lights
`lights` turns a button (any binary sensor) into a wall dimmer for a light of a remote node: a short press toggles the
light, holding the button ramps its brightness:

```yaml
canopen_entities:
  remote_nodes:
    dimmer3: dimmer3.yaml

canopen_buttons:
  lights:
    - button: in6
      node: dimmer3
      entity: light4
```

* `button` (Required, ID): the button, any binary sensor. Its own `on_press` / `on_release` etc. still run.
* `node` (Required, string): name of the node in `remote_nodes`
* `entity` (Required, string): id of the remote light (with brightness, and `tpdo`). A light without brightness
  fails validation: use `canopen_entities.light.toggle` for it.
* `hold_time` (Optional, time, default `400ms`): a shorter press toggles the light (sent on release), a longer one
  dims it
* `ramp_time` (Optional, time, default `3s`): time to ramp the brightness over the full range
* `step_interval` (Optional, time, default `100ms`, at least `20ms`): a brightness command is sent this often while
  dimming
* `min_brightness` (Optional, percentage, default `2%`): dimming down stops here, it doesn't switch the light off

Direction of a hold: up from `min_brightness` if the light is off (it is switched on), up at the minimum, down at full
brightness, otherwise the opposite of the previous hold. The light's state and brightness at the start come from
[remote state slots](#remote-states), during the ramp the button counts on its own. Each step is a separate
brightness command: the remote light should have `default_transition_length: 0s`, otherwise every step starts a
transition and the light lags behind the button.

Until 2026-11-21 this was `canopen_entities: light_buttons:`, with `binary_sensor:` instead of `button:`; that key
now fails validation with a pointer here.

### Covers
`covers` moves a cover of a remote node while a button is held: one button opens, another closes, releasing stops:

```yaml
canopen_buttons:
  covers:
    - open_button: in1
      close_button: in2
      node: martin-c3
      entity: cover_hall
```

* `open_button` (Optional, ID): binary sensor opening the cover while held
* `close_button` (Optional, ID): binary sensor closing the cover while held (at least one of `open_button`,
  `close_button`)
* `node` (Required, string): name of the node in `remote_nodes`
* `entity` (Required, string): id of the remote cover (the OD writer on; `tpdo` isn't needed)

The last press wins: pressing the other button while one is held reverses the cover, and releasing one while the
other is still held moves the other way. When no button is held the cover stops. The stop is sent twice, 50 ms apart:
commands get no reply, and a lost stop would run the cover to its end. At an end position the cover stops by itself.

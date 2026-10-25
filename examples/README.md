# Examples

| File | What it shows |
|---|---|
| `light.yaml` | node 7 with three lights (cwww, monochromatic, binary), all broadcasting their state in TPDO 0 |
| `light-switch.yaml` | node 8 controlling the lights of `light.yaml` over CAN: `remote_nodes`, `platform: canopen` remote-state entities, `canopen.light.turn_on` / `turn_off` / `toggle` actions (see [Linking nodes](../docs/linking-nodes.md)) |
| `light_cwww.yaml` | a single cold/warm white light |
| `venetian_blinds.yaml` | a cover with tilt |
| `misc.yaml` | sensor, binary sensor, switch, cover and light on one node, in two TPDOs |
| `common.yaml` | shared settings included by all examples as a package (board, WiFi, API, canbus) |

## Building

```
cp wifi.yaml.template wifi.yaml   # and put your WiFi credentials in it
esphome config light.yaml
esphome run light.yaml
```

`common.yaml` targets an ESP32-C3 and uses the `canbus_udp_multicast` virtual CAN bus
([esphome-canbus-udp-multicast](https://github.com/mrk-its/esphome-canbus-udp-multicast)). With it, nodes and
can2mqtt talk over UDP multicast on your LAN, so no CAN hardware is needed for testing. For a real bus, switch to the
commented-out `esp32_can` block and set the pins and bit rate.

The examples load the component from `../components` (this repository). In your own configs use
`source: github://mrk-its/esphome-canopen`.

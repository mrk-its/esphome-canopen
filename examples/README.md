# Examples

* `light.yaml` defines node with 2 lights, mapped to CANOpen entities. These lights broadcast their state / brightness using TPDO#0 messages. Lights are exposed as CANOpen entities and through ESP API

* `light-switch.yaml` acts as light controller and shows how to control entities on other CANOpen nodes. It tracks state of lights defined by `light` node using RPDO mapping: TPDO messages carring lights state are mapped to locally-defined entities. Light state is changed with `send_entity_cmd` call.

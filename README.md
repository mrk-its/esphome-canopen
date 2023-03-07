# ESPHome CANopen external component

This project provides ESPHome external_component named `canopen` for easy conversion of ESPHome entities into CANopen nodes. It allows to map selected entities to CANopen object dictionary. It also allows to define TPDOs for sending entity state on change. Currently following entity types are supported: `sensor`, `binary_sensor`, `switch` and `cover`. Entity metadata is also published in Object Dictionary enabling autodiscovery.

The [can2mqtt](https://github.com/mrk-its/can2mqtt) project provides can2mqtt bridge exposing ESPHome CANopen entities onto MQTT topics. It follows MQTT Discovery protocol, so entities appear automatically in HomeAssistant.


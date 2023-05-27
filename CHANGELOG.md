# 2024-05-27, v0.3.0
* add support for heartbeat consumers
* fix serious timer driver bug


# 2023-05-23, v0.2.1
* add support for up to 8 TPDOs (TPDO 4..7 use RPDO 0..3 CAN ids)
* heartbeat interval is configurable via canopen.heartbeat_interval configuration property
* `state` enum added to `status_info` struct
* new methods: `initiate_recovery()` and `start()` for performing recovery from bus_error state
* esphome.name exposed as manufacturer device name (OD: 0x1008)
* hw_version (OD: 0x1009) and sw_version (OD: 0x100a) configurable via `hw_version` / `sw_version` config properties
* all esphome-devices have the same Ientity VendorId / ProductCode magic values (used by can2mqtt)
* added support for Store Parameters (OD: 0x1010) and Restore default parameters (OD: 0x1011), used for storing RPDO configuration

# 2023-05-13, v0.1.0
* support for packing sensor data into 8-bit or 16-bit unsigned integers

  Is it possible now to scale and pack sensor value into 8-bit / 16-bit unsigned integer, to reduce size in TPDO frame. Sensor value is scaled to 8-bit or 16-bit width before transmission and scaled back on receive. Following additional parameters are recognized:
  * size (in bytes), valid values are 1 (8-bit int) or 2 (16-bit int)
  * min_value: min_value of sensor, used for scaling
  * max_value: max value of sensor, used for scaling
  values outside of range are clipped into [min_value, max_value]

* configure canopen early, with setup_priority::HARDWARE
* fix 'oscillations' caused by mutual state updates by delaying state update by configured amount of time (50ms by default, can be changed via `state_update_delay` config option)
* add `trig_tpdo` method to trigger TPDO

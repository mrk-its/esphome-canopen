# esphome-canopen

ESPHome external component exposing ESPHome entities over CANopen, plus CAN OTA. The host side is
[can2mqtt](https://github.com/mrk-its/can2mqtt) (MQTT/Home Assistant bridge, also the OTA uploader).
OD layout for entities: see OBJECT_DICTIONARY.md.

## Layout
- `components/canopen/__init__.py`: config schema and codegen. Global build flags for the stack
  (`-DCO_SDO_BUF_SEG`, `-DCO_TPDO_N`, `-DCO_RPDO_N`, `-DCO_SSDO_N`, `OTA_COMPRESSION`, ...) are added in `to_code()`.
- `layout.py`: entity states and TPDO layout computed from YAML (single source of truth; C++ gets caps from
  codegen and checks the layout hash at boot, OD `0x3002:1`). ESPHome-free, tested by `test/test_layout.py`.
- `remote.py`: `remote_nodes` - loads another node's YAML (packages/substitutions/extend passes only, no
  validation) to resolve `rpdo: {node, entity}`, `platform: canopen` entities and the `canopen.*` actions.
  Raw YAML values may be strings after substitution: coerce (`_normalize_node_config`, `layout.flag`).
- Remote state slots: remote states read by `platform: canopen` entities (`binary_sensor.py`, `sensor.py`,
  `text_sensor.py`, C++ in `remote_entities.*`) and toggle actions are collected from the full validated config
  (`remote_state_slots()`, entries marked with `CONF_REMOTE_STATE`), OD `0x4000+n:1`, filled by RPDO.
  Typed actions `canopen.<domain>.<action>` (`_remote_action()`) -> `RemoteCommandAction` writes.
- `canopen.cpp/.h`: node setup, OD construction (`od.add_update(...)`), main `loop()`.
- `driver_can.cpp`: glue between canopen-stack's CAN/timer/NVM driver interface and ESPHome `canbus`.
  Frames arrive via the canbus callback (`on_frame`) and are processed immediately (`CONodeProcess`).
- `fw.cpp/.h`: OD object types for firmware upload (`FwImage*`, `FwCtrl*`).
- `ota/`: the `ota: - platform: canopen` component (`CanopenOTAComponent`), wraps the platform's
  `esphome::ota::make_ota_backend()`; optional zlib (miniz) decompression.
- canopen-stack itself is pulled in with `cg.add_library("canopenstack", None, "https://github.com/mrk-its/canopen-stack#test")`.
  Its sizes are configured only through `-DCO_*` build flags (defaults in its `src/config/co_cfg.h`).

## CAN OTA flow
- OD 0x3000: `:1` ctrl, `:2` size, `:3` MD5 (16 bytes), `:4` image data (SDO block download), `:5` flags
  (bit 0 = node expects zlib-compressed data).
- can2mqtt `firmware_upload()` writes size, MD5 (of the uncompressed image), then the image with block
  transfer, compressing only if flag bit 0 is set.
- `FwImageReset` -> `ota->begin(size)`, `FwImageWrite` -> `ota->write()`, last chunk -> `ota->end(md5)`.
  `end()` is deferred (`ota_finish()` + `loop()` states, ~1 s each) because on some platforms
  `backend->end()` reboots immediately and the SDO block transfer must finish first.
- The SDO block size the node announces is `sdo_block_transfer_size` (-> `CO_SDO_BUF_SEG`). The node
  decides it, not can2mqtt (canopen-asyncio follows the server's blksize).

## Platform notes
- Upstream ESPHome compatibility: the component must load on upstream ESPHome (ESP32/ESP8266 users) as well
  as the esphomeplatformzephyr fork. Fork-only APIs (e.g. `SWAP_METHOD_SCHEMA`,
  `esphome.components.zephyr.mcuboot`) are imported in `try/except ImportError` or lazily; use
  `getattr(CORE, "is_zephyr", False)` / `is_stm32`.
- `OTA_COMPRESSION` is off on stm32 and `platform: zephyr` (miniz inflate needs ~43 KB of heap).
- `platform: zephyr` (esphomeplatformzephyr fork):
  - OTA needs `zephyr: advanced: bootloader: mcuboot` and a slot0/slot1 partition layout. The backend
    (`ota_backend_zephyr.cpp`) writes slot1, checks MD5, `boot_request_upgrade(BOOT_UPGRADE_TEST)`;
    `safe_mode` confirms the image after 60 s of uptime, otherwise MCUboot rolls back.
  - The OTA payload must be `.esphome/build/<node>/.west_build/zephyr/zephyr/zephyr.signed.bin`
    (starts with MCUboot magic `3d b8 f3 96`). `ota.cpp` rejects other images on the first chunk.
  - `swap_method:` on the canopen OTA platform is passed to `apply_swap_method()`.
  - `-DCO_*` flags only reach canopen-stack with esphomeplatformzephyr PR #196 (project build flags
    for converted libraries). Without it the stack uses its `co_cfg.h` defaults (e.g. 127 SDO segments)
    while the app uses the configured values: mismatched buffer sizes and struct layouts. Check with
    `compile_commands.json` that `co_ssdo.c` and `canopen.cpp` get the same `-DCO_*` flags.
  - `zephyr_can` RX queue defaults to 8 frames: set `rx_buffer_size` (max 64) to at least the SDO block size.

## Testing
- ESP32 build in Docker: `./docker_build.sh` (renders `test/config.yaml.template` with `envsubst`; note
  `framework` must be a mapping, e.g. `{type: esp-idf}`).
- Config validation without building: `esphome config <yaml>`, against both upstream ESPHome and the
  esphomeplatformzephyr venv, when touching Python schema code.
- Zephyr hardware example: `zephyr-examples/stm32l4/stm32l4_blinking_led.yaml` (nucleo_l476rg, MCUboot,
  `ota: - platform: canopen`, `external_components: source: ../../esphome-canopen/components`).

## Debugging CAN OTA
- Serial log 115200: `FwImageWrite, N kB (P%)` every KB; measure speed from its timestamps.
- At DEBUG, `canbus` logs every received frame. Over a 115200 UART that is slower than the frames arrive
  during block transfer (queue overflows). Set `logger: logs: canbus: INFO` (also `script`) on nodes doing OTA.
- A perfectly regular, very slow rate (e.g. 16.6 s/KB) independent of block size means frames are lost
  in every block and each loss costs a 2 s SDO timeout + retransmit (seen with block size 32/63 via the
  esp_usb_serial bridge, where only ~16 frames per burst got through). Block size 16 gave ~1.5 KB/s.
  can2mqtt logs `Will start retransmission` only at DEBUG, and reports HA progress only for fully acked blocks.
- `zephyr_can` `rx errors` bursts point at the physical bus (stub length, termination), not the node.

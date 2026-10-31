# esphome-canopen

ESPHome external component exposing ESPHome entities over CANopen, plus CAN OTA. The host side is
[can2mqtt](https://github.com/mrk-its/can2mqtt) (MQTT/Home Assistant bridge, also the OTA uploader).
OD layout for entities: see OBJECT_DICTIONARY.md.
User docs: README.md is the overview / quick start only; the option reference lives in `docs/configuration.md`,
`docs/linking-nodes.md`, `docs/ota.md`, `docs/remote-logs.md`. Document new options there.
Port to CANopenNode (replacing canopen-stack), in progress: plan and handoff state in `docs/canopennode.md`,
current step in `docs/device_api_canopennode.md` (step 1: `docs/minimal_canopennode.md`, done; step 3:
`docs/sdo_server_canopennode.md`). Upstream PRs 1-3 as built and tested: `docs/upstream_prs_1-3.md`. Hub PRs 7+
(generic CiA 301 for `canopen_entity_mapper`) are developed on branch `canopen-next` in `~/repos/esphome`
("Hub for the entity mapper" in `docs/canopennode.md`; PRs 7-9 (SDO server, OD API, own PDOs) done,
PR 10 store/restore next).

## Workflow
- Always do new features and fixes on a `feature/<short-name>` branch (created from the current branch,
  usually `dev`), never directly on `dev` or `main`.

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
- `logs.cpp/.h`: remote logs (`remote_logs:` -> `USE_CANOPEN_LOGS`), OD 0x3100, global `canopen_logs` (static
  ring buffer) fed by a logger callback registered in `start_logs()`, which codegen calls before `App.setup()` (so
  the setup logs of all components are captured).
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

## Remote logs
- Spec: `docs/can-logs-plan.md`. Host polls (Log Pending `0x3100:2`, then uploads Log Data `:1`), writes Log Level
  `:3` as subscribe/keepalive; the node turns itself off after `timeout` without polls.
- `LogDataSize` / `LogDataReset` snapshot `pending` into `xfer_remaining`; `LogDataRead` never consumes more than that
  (the last block upload passes `SegNum * 7`, not the remaining size). Overflow drops the newest line, marker
  `[W][canopen.logs]: N messages dropped` in the stream.
- No locking: log callbacks and SDO processing both run on the main loop. Nothing in `logs.cpp` may log.
- Log upload block size: `remote_logs: block_size:` (default 3 = STM32 TX mailboxes only), advertised in `0x3100:7`,
  requested by can2mqtt (3 if `:7` is missing). The node sends a whole block back to back, so it must fit the TX
  queue (not detected from the canbus platform). It must be <= `sdo_block_transfer_size`, which is the SDO buffer:
  download block size (OTA, RX side) and cap for upload blocks.

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
  - `zephyr_can` software TX queue (`tx_buffer_size`, default 16; esphomeplatformzephyr branch
    `feature/zephyr-can-tx-queue`): without it only the 3 TX mailboxes hold outgoing frames. Size
    `remote_logs: block_size:` to fit it (leave room for 8 TPDOs + heartbeat).

## Testing
- `pytest test/`: Python tests, plus `test/test_logs.py`, which builds `test/logs/logs_test.cpp` against
  `components/canopen/logs.cpp` with stub headers (`test/logs/stubs/`) and runs it (ASan/UBSan if available), and
  `test/test_od.py` (same for `components_next/canopen/od.cpp` against CANopenNode's `CO_ODinterface.c`; needs
  `../CANopenNode` or `$CANOPENNODE_PATH`, skipped otherwise) and `test/test_frames.py` (the heartbeat/EMCY
  decoders in `components_next/canopen/types.h`). pytest is in the esphomeplatformzephyr venv.
- Upstream ESPHome (`dev` from GitHub) venv: `~/repos/esphome-upstream-venv` (update with
  `bin/pip install -U "esphome @ git+https://github.com/esphome/esphome@dev"`). Use it, not the esphomeplatformzephyr
  venv, to check upstream compatibility (the fork may carry APIs upstream doesn't have). Upstream PRs target `dev`,
  so `components_next` may use APIs newer than the last release (e.g. `canbus::send_data(..., data, len)`, in `dev`
  but not in 2026.9.x).
- `components_next` (CANopenNode) build checks with it: `test/esp32_canopen.yaml` (ESP32-C3, esp-idf; add
  `-s framework arduino` for Arduino) and `test/esp8266_canopen.yaml` (d1_mini + mcp2515), e.g.
  `~/repos/esphome-upstream-venv/bin/esphome compile test/esp32_canopen.yaml`. CANopenNode comes from GitHub
  (`CANOPENNODE_SOURCE` in `components_next/canopen/__init__.py`, the fork's `library-json` branch (upstream PR #664) pinned to a commit;
  bump it after pushing the fork); `canopennode_path: <local checkout>` overrides it for development.
- Old component ESP32 build in Docker: `./docker_build.sh` (renders `test/config.yaml.template` with `envsubst`;
  note `framework` must be a mapping, e.g. `{type: esp-idf}`). Docker leaves root-owned files in `.esphome/`.
- Config validation without building: `esphome config <yaml>`, against both upstream ESPHome (the venv above) and
  the esphomeplatformzephyr venv, when touching Python schema code.
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

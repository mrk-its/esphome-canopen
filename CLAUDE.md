# esphome-canopen

ESPHome external component `canopen_entities` (`components/canopen_entities`), exposing ESPHome entities over
CANopen, plus CAN OTA and remote logs. It sits on the `canopen` hub (generic CiA 301 on CANopenNode, going upstream;
developed in `~/repos/esphome`, branch `canopen-full`). The host side is
[can2mqtt](https://github.com/mrk-its/can2mqtt) (MQTT/Home Assistant bridge, also the OTA uploader).
OD layout for entities: see OBJECT_DICTIONARY.md.
The old component (`components/canopen`, on canopen-stack) is only on `main` (removed on `feature/entity-mapper`,
2026-11-01); nodes still running it interoperate (same OD, `remote.py` reads their YAML).
User docs: README.md is the overview / quick start only; the option reference lives in `docs/configuration.md`,
`docs/linking-nodes.md`, `docs/ota.md`, `docs/remote-logs.md`, old -> new in `docs/migration.md`. Document new options
there. The hub's own options are documented in esphome-docs (`~/repos/esphome-docs`, branch `canopen-full`); the user
docs link its `canopen.mdx` on GitHub and the hub source `github://mrk-its/esphome@canopen-full` (pushed; keep it pushed after hub changes).
Port to CANopenNode (replacing canopen-stack), in progress: plan and handoff state in `docs/canopennode.md`,
current step in `docs/device_api_canopennode.md` (step 1: `docs/minimal_canopennode.md`, done; step 3:
`docs/sdo_server_canopennode.md`). Upstream PRs 1-3 as built and tested: `docs/upstream_prs_1-3.md`. Hub PRs 7+
(generic CiA 301 for `canopen_entities`) are developed on branch `canopen-next` in `~/repos/esphome`
("Hub for the entity mapper" in `docs/canopennode.md`; PRs 7-12 (SDO server, OD API, own PDOs, store/restore,
loopback between local nodes, SDO block transfer) done).
Upstream PRs are being re-split (the old `canopen-1..4` branches / `docs/upstream_prs_1-3.md` are superseded): the hub
refactored by feature on branch `canopen-full` in `~/repos/esphome`, the file -> PR map, line counts and cutting
steps in `docs/canopennode.md`, "Upstream PRs" -> "New split"; per-PR line counts with `scripts/pr_split_lines.py`.
Planned after PR 6: OTA in the hub on CiA 302-3 program download objects (`0x1F50`/`0x1F51`/`0x1F56`/`0x1F57`) instead
of the mapper's `0x3000`: `docs/canopen_ota.md`.

## Workflow
- Always do new features and fixes on a `feature/<short-name>` branch (created from the current branch,
  usually `dev`), never directly on `dev` or `main`.

## Layout
`components/canopen_entities/`:
- `__init__.py`: config schema and codegen, on top of the hub (`canopen_id`; hub hooks `canopen.enable_pdo()`,
  `enable_sdo_block()`, `register_node()`...). Entity metadata / states / commands at `0x2000 + 16 * N`, the ESPHome
  identity can2mqtt checks, the OD writer.
- `layout.py`: entity states and TPDO layout computed from YAML (single source of truth; C++ gets caps from
  codegen and checks the layout hash at boot, OD `0x3002:1`). ESPHome-free, tested by `test/test_layout.py`.
- `remote.py`: `remote_nodes` - loads another node's YAML (packages/substitutions/extend passes only, no
  validation; both formats: `canopen_entities:` and the old component's `canopen: entities:`) to resolve
  `rpdo: {node, entity}`, `platform: canopen_entities` entities and the `canopen_entities.<domain>.<action>` actions.
  Raw YAML values may be strings after substitution: coerce (`_normalize_node_config`, `layout.flag`).
- Remote state slots: remote states read by `platform: canopen_entities` entities (`binary_sensor.py`, `sensor.py`,
  `text_sensor.py`, C++ in `remote_entities.*`) and toggle actions are collected from the full validated config
  (`remote_state_slots()`, entries marked with `CONF_REMOTE_STATE`), OD `0x4000+n:1`, filled by RPDO.
  Typed actions `canopen_entities.<domain>.<action>` (`_remote_action()`; `canopen.<domain>.<action>` on the old
  component and until 2026-11-01) -> `RemoteCommandAction` writes.
- `canopen_entities.cpp/.h`: `CANopenEntities` (the component), `MappedEntity` (one entity, the `OdObject` handler of
  its three records), remote state slots, the OD writer.
- `log_buffer.*` / `remote_logs.*`: remote logs, see "Remote logs".
- `ota/`: the `ota: - platform: canopen_entities` component, OD `0x3000`, wraps the platform's
  `esphome::ota::make_ota_backend()`; optional zlib (miniz) decompression. Planned to move into the hub on CiA 302-3
  objects (`docs/canopen_ota.md`).
- The hub (`canopen`) comes from `~/repos/esphome` (`external_components` with `components: [canopen]`); CANopenNode is
  pinned there (`CANOPENNODE_SOURCE` in the hub's `__init__.py`, the fork's `library-json` branch (upstream PR #664);
  bump it after pushing the fork; `canopennode_path: <local checkout>` overrides it for development).

## CAN OTA flow
- OD 0x3000: `:1` ctrl, `:2` size, `:3` MD5 (16 bytes), `:4` image data (SDO block download), `:5` flags
  (bit 0 = node expects zlib-compressed data).
- can2mqtt `firmware_upload()` writes size, MD5 (of the uncompressed image), then the image with block
  transfer, compressing only if flag bit 0 is set.
- One `OdObject` handler (`ota/ota.cpp`): the first chunk of `:4` -> `ota->begin(size)` (size from `:2`), every chunk
  -> `ota->write()`, the last one (a domain's stream size is 0 until the last chunk: CANopenNode doesn't pass the
  client's indicated size) sets the MD5. Backend `end()` and the reboot are deferred to `loop()` (~1 s each) because on
  some platforms `backend->end()` reboots immediately and the SDO block transfer must finish first.
- The SDO block size the node announces is the hub's `sdo_block_size` (`DEFAULT_SDO_BLOCK_SIZE` 16 if unset). The node
  decides it, not can2mqtt (canopen-asyncio follows the server's blksize).

## Remote logs
- Spec: `docs/can-logs-plan.md`. Host polls (Log Pending `0x3100:2`, then uploads Log Data `:1`), writes Log Level
  `:3` as subscribe/keepalive; the node turns itself off after `timeout` without polls.
- `canopen_entities: remote_logs:` (the old component's options and OD). `log_buffer.*` is the ring buffer / level /
  keepalive logic without OD glue (host test `test/logs/log_buffer_test.cpp`), `remote_logs.*` the 0x3100 handler:
  the snapshot of `pending` is taken at the first read (`offset == 0`), reads copy at most the rest (CANopenNode asks
  for its free buffer space). Overflow drops the newest line, marker `[W][canopen.logs]: N messages dropped` in the
  stream. Tag filter uses `ESPHOME_strcmp_P` (tags in flash on ESP8266).
- No locking: log callbacks and SDO processing both run on the main loop. Nothing in `log_buffer.cpp` /
  `remote_logs.cpp` may log.
- Log upload block size: `remote_logs: block_size:` (default 3 = STM32 TX mailboxes only), advertised in `0x3100:7`,
  requested by can2mqtt (3 if `:7` is missing). The node sends a whole block back to back, so it must fit the TX
  queue (not detected from the canbus platform). Not capped by the hub's `sdo_block_size` (a download cap; the
  900-byte buffer takes 127 segments).

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
    (starts with MCUboot magic `3d b8 f3 96`). `ota/ota.cpp` rejects other images on the first chunk.
  - `swap_method:` on the `canopen_entities` OTA platform is passed to `apply_swap_method()`.
  - The hub's `-DCO_CONFIG_*` flags only reach CANopenNode with esphomeplatformzephyr PR #196 (project build flags
    for converted libraries). Without it the library uses the port header's defaults while the app uses the
    configured values: mismatched struct layouts. Check with `compile_commands.json` that CANopenNode's sources and
    the hub get the same flags.
  - `zephyr_can` RX queue defaults to 8 frames: set `rx_buffer_size` (max 64) to at least the SDO block size.
  - `zephyr_can` software TX queue (`tx_buffer_size`, default 16; esphomeplatformzephyr branch
    `feature/zephyr-can-tx-queue`): without it only the 3 TX mailboxes hold outgoing frames. Size
    `remote_logs: block_size:` to fit it (leave room for 8 TPDOs + heartbeat).

## Testing
- `pytest test/`: Python tests, plus `test/test_logs.py`, which builds `test/logs/log_buffer_test.cpp` against
  `components/canopen_entities/log_buffer.cpp` with stub headers (`test/logs/stubs/`) and runs it (ASan/UBSan if
  available). `test/test_canopen_entities.py` runs codegen / validation through the upstream venv with the hub from
  `~/repos/esphome` (`$ESPHOME_UPSTREAM_VENV`, `$ESPHOME_CANOPEN_HUB`; skipped without them). `test/remote_node.yaml`
  is a node in the old component's format, the fixture of `test_mapper_remote.py`. pytest is in the
  esphomeplatformzephyr venv.
- The hub's own tests are in `~/repos/esphome`: `script/cpp_unit_test.py canopen`,
  `script/test_build_components.py -c canopen -t <target>` (see `docs/canopennode.md`).
- Upstream ESPHome (`dev` from GitHub) venv: `~/repos/esphome-upstream-venv` (update with
  `bin/pip install -U "esphome @ git+https://github.com/esphome/esphome@dev"`). Use it, not the esphomeplatformzephyr
  venv, to check upstream compatibility (the fork may carry APIs upstream doesn't have). Upstream PRs target `dev`,
  so the hub may use APIs newer than the last release (e.g. `canbus::send_data(..., data, len)`, in `dev` but not in
  2026.9.x).
- Build checks: `test/esp8266_mapper.yaml` (upstream venv, d1_mini + mcp2515), `test/stm32l4_canopen.yaml`,
  `test/stm32l4_biuro_switch.yaml`, `test/stm32l4_loopback.yaml` (fork venv, nucleo_l476rg), all with the hub from
  `../../esphome` (whatever branch is checked out there).
- Config validation without building: `esphome config <yaml>`, against both upstream ESPHome (the venv above) and
  the esphomeplatformzephyr venv, when touching Python schema code.
- CI (`.github/workflows/test.yml`): pytest only (codegen tests skipped). No build job yet; one could take the hub
  from mrk-its/esphome `canopen-full` (pushed) with ESPHome `dev`.

## Debugging CAN OTA
- Serial log 115200: `Received N kB (P%)` every KB; measure speed from its timestamps.
- At DEBUG, `canbus` logs every received frame. Over a 115200 UART that is slower than the frames arrive
  during block transfer (queue overflows). Set `logger: logs: canbus: INFO` (also `script`) on nodes doing OTA.
- A perfectly regular, very slow rate (e.g. 16.6 s/KB) independent of block size means frames are lost
  in every block and each loss costs a 2 s SDO timeout + retransmit (seen with block size 32/63 via the
  esp_usb_serial bridge, where only ~16 frames per burst got through). Block size 16 gave ~1.5 KB/s.
  can2mqtt logs `Will start retransmission` only at DEBUG, and reports HA progress only for fully acked blocks.
- `zephyr_can` `rx errors` bursts point at the physical bus (stub length, termination), not the node.

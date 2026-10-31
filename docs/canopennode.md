# Porting esphome-canopen to CANopenNode

Status: plan. The first step is detailed in [minimal_canopennode.md](minimal_canopennode.md).

## Goals
- Replace canopen-stack (`cg.add_library("canopenstack", None, "https://github.com/mrk-its/canopen-stack#test")`)
  with [CANopenNode](https://github.com/CANopenNode/CANopenNode) v4.
- Separate the CANopen node from the ESPHome entity mapper. The node layer must also be usable on its own,
  to talk to third-party CANopen devices (SDO client, NMT, heartbeat, EMCY), without `entities:`.
- **Wire compatibility with can2mqtt stays as it is:**
  - the OD layout (`OBJECT_DICTIONARY.md`: entities at `0x2000+16n`, remote state slots at `0x4000+n`,
    `0x3000` OTA, `0x3002` layout hash, `0x3100` logs);
  - the TPDO layout (`components/canopen/layout.py`);
  - the OD-writer PDO;
  - store/restore of the comm params (`0x1010`/`0x1011`).
- Only one backend (CANopenNode); canopen-stack is removed at the end. The abstraction stays thin: mapper and
  services use a stack-neutral API, but there is no runtime choice of backend.

## Current state (handoff)
- **esphome-canopen:**
  - branch `feature/canopennode`, created from `main` at `8696ae8`;
  - test config: `test/stm32l4_canopen.yaml`.
  - step 1 (`docs/minimal_canopennode.md`, NMT + heartbeat) is implemented in `components_next/canopen/`. Tested on the nucleo: `CO_CANopenInit` OK, NMT operational, HB consumer sees node 0x01 (5 s
    heartbeat) and fires `on_hb_consumer_event` on timeout, the bus stays error-active with 0 TX errors.
  - `-DCO_*` build flags reach the app build of the library. The library is also compiled for the MCUboot image
    (without the project flags), because esphomeplatformzephyr `build_zephyr.py` passes the global
    `-DEXTRA_ZEPHYR_MODULES`; `--gc-sections` drops all of it from the bootloader binary. Fix in the fork: pass
    `-D<app image>_EXTRA_ZEPHYR_MODULES` (image `zephyr`, the app dir name) so only the app gets the libraries.
  - CLAUDE.md workflow: new work goes on `feature/<name>` branches, never directly on `dev`/`main`. Commit only when
    asked.
- **CANopenNode fork:**
  - local checkout `~/repos/CANopenNode` (origin `git@github.com:mrk-its/CANopenNode.git`);
  - all on top of upstream master `9b8beed` (2026-07-10), pushed:
    - branch `library-json`, commit `3fb805b` "Add library.json": packaging only, proposed upstream as CANopenNode PR
      #664. Builds `CANopen.c`, `301`, `303`, `304`, `305`, `309`, `extra` and `storage/CO_storage.c`; the services
      our port config disables are dropped by the linker (no size change);
    - branch `sdo-srv-blksize`, commit `67ad4ba`: the `CO_CONFIG_SDO_SRV_BLKSIZE` patch alone (below), to propose
      upstream separately;
    - branch `esphome`, commit `9ff367b` (2026-10-30: reset to `library-json` + cherry-pick of `67ad4ba`, force-pushed;
      was `f38ec8a`, library.json only): what every build pins (`CANOPENNODE_SOURCE` in the PR branches,
      `canopen-next` and `components_next`);
    - branch `esphome-old-dce5a01`: the old all-in-one commit (library.json + port header + block-size patch), kept
      for reference.
- **Port header** `components_next/canopen/canopennode_port/CO_driver_target.h` (moved out of the fork, 2026-10-29).
  Project `-I` flags don't reach converted libraries on the native backends (only `-D`/`-W` do,
  `get_project_compile_flags()` in `esphome/framework_helpers.py`), so codegen adds the directory as a header-only
  package, one way per backend:
  - **ESP32 (ESP-IDF and Arduino, both native IDF builds):** `add_idf_component(name="canopennode_port",
    path=...)` (`canopennode_port/CMakeLists.txt`: `idf_component_register(INCLUDE_DIRS ".")`). A converted library
    only sees the components it REQUIRES, which include every project-managed component
    (`ESPHOME_PROJECT_MANAGED_COMPONENTS`). A local IDF component is named after its directory, hence the
    directory name;
  - **Zephyr (native west build):** a header-only local library (`canopennode_port/library.json`,
    `"CANopenNode-esphome-port"`, `includeDir: "."`, no sources); Zephyr exports every library's include dirs
    globally;
  - **PlatformIO (ESP8266, ...):** `-I<dir>` as a project build flag (PlatformIO passes build flags to libraries; its
    LDF doesn't link the library above to CANopenNode). Added together with the library for every non-ESP32
    target.

  Checked (2026-10-29): Zephyr (esphomeplatformzephyr venv, 2026.11.0-dev) builds and runs on the nucleo (NMT
  operational, node 0x01 heartbeat seen, bus error-active with 0 TX errors); upstream ESPHome 2026.9.1 in Docker
  builds `test/esp32_canopen.yaml` (ESP32-C3, esp-idf: RAM 52.7 KB, flash 227 KB; `-s framework arduino`: 52.4 KB /
  243 KB) and `test/esp8266_canopen.yaml` (d1_mini + mcp2515: 30.2 KB / 292 KB) with a vector-based `send_data()`;
  upstream `dev` (`95973f0`) in the venv builds all three with the `(data, len)` one (52.2 KB / 228 KB, 52.0 KB /
  244 KB, 29.3 KB / 287 KB). Not run on ESP32/ESP8266 hardware.
  The header contains:
    - `CO_MULTIPLE_OD`;
    - default config, each value in `#ifndef` so it can be overridden with `-D`:
      - NMT with `CALLBACK_CHANGE | MASTER`;
      - HB consumer with `ENABLE | CALLBACK_MULTI | QUERY_FUNCT`;
      - EM `PRODUCER`;
      - SDO server segmented + block with buffer 900;
      - SDO client enabled + segmented + block with buffer 1000;
      - PDO RPDO/TPDO, timers and `OD_IO_ACCESS`;
      - CRC16 and FIFO (`ALT_READ`, `CRC16_CCITT`);
      - TIME/SYNC/LEDs/LSS/GFC/SRDO/GTW/TRACE/STORAGE/NODE_GUARDING off;
    - the types `CO_CANrxMsg_t {uint32_t ident; uint8_t DLC; uint8_t data[8];}` + `CO_CANrxMsg_read*` macros,
      `CO_CANrx_t`, `CO_CANtx_t`, `CO_CANmodule_t` (`CANptr` = the ESPHome component), `CO_storage_entry_t`;
    - no-op `CO_LOCK_*`;
- **Block-size patch** (branch `sdo-srv-blksize`), `301/CO_SDOserver.h/.c`: new `CO_CONFIG_SDO_SRV_BLKSIZE` (1..127, default 127, checked with `#error`) replaces
    the hard-coded 127 cap in the block-download initiate and sub-block responses. The ≥ 900 buffer requirement is
    unchanged.

  Verified (with the old layout): `CANopen.c` and all `301/*.c` compile with gcc on the host for BLKSIZE 127 and 16
  with no warnings (`-Wall -Wextra`). BLKSIZE 200 is rejected. clangd "file not found" diagnostics in the fork are
  only the editor missing include paths.

## Decisions
- **Library source:** the fork from GitHub, `cg.add_library("CANopenNode", None, CANOPENNODE_SOURCE)` with
  `https://github.com/mrk-its/CANopenNode.git#<commit>` (branch `esphome`, pinned to a commit so a push to the fork
  doesn't change builds; bump it in `__init__.py`).
  - `canopennode_path: <absolute path>` (optional) uses a local checkout instead (`file://`), for development.
  - The esphomeplatformzephyr library converter (`esphome/platformio/library.py`, Zephyr backend
    `esphome/components/zephyr/library.py`) works like this:
    - it takes registry, git, url or `file://` sources (`LocalSource`; **relative `file://` is rejected**);
    - it requires `library.json`/`library.properties` on ESP-IDF and Zephyr (`manifest_optional` is only set for
      host builds);
    - it turns each library into a Zephyr module wired in through `EXTRA_ZEPHYR_MODULES`;
    - project `cg.add_build_flag()` flags reach the library through `ESPHOME_LIBRARY_COMPILE_OPTIONS`;
    - include dirs (`includeDir`, `srcDir`, the library's own `-I` flags) are exported globally; project `-I`
      flags don't reach libraries at all (hence the port header as its own header-only library).
  - Done 2026-10-29: the default is the git URL (checked: Zephyr, ESP32 esp-idf, ESP8266 clone `f38ec8a` and build;
    now `9ff367b`, see "CANopenNode fork").
- **Block size: `sdo_block_transfer_size` dropped for now (2026-10-29), add it back later.** It needs
  `CO_CONFIG_SDO_SRV_BLKSIZE` (the fork patch, option "D"), which caps the block-download size the SDO server
  announces; without it (default 127) CANopenNode announces 127. Since 2026-10-30 the patch is in the `esphome` branch
  the build uses (`9ff367b`), so the option can come back (hub PR 12: 1..127, default 63 →
  `-DCO_CONFIG_SDO_SRV_BLKSIZE`, plus block transfer in the server, buffer >= 900); for upstream ESPHome the patch
  should still go to CANopenNode upstream (branch `sdo-srv-blksize`). Needed at the latest for CAN OTA
  (step 6), likely already for block download in step 3:
  - We need the node to choose (`sdo_block_transfer_size`): `zephyr_can` keeps at most 64 RX frames, and the
    esp_usb_serial bridge got only about 16 frames per burst through.
  - can2mqtt follows the size the server announces.
  - Could be proposed upstream.
- **Driver in the component, not in the library.**
  - The CANopenNode driver functions (declared in `301/CO_driver.h` inside `extern "C"`) are implemented in the
    component on top of ESPHome `canbus`: `CO_CANsetConfigurationMode`, `CO_CANsetNormalMode`, `CO_CANmodule_init`,
    `CO_CANmodule_disable`, `CO_CANrxBufferInit`, `CO_CANtxBufferInit`, `CO_CANsend`, `CO_CANclearPendingSyncPDOs`,
    `CO_CANmodule_process`.
  - `CANptr` is the component, so there are no globals (`current_canopen` / `all_instances` go away).
  - A frame that can't be sent now (`canbus::ERROR_ALLTXBUSY` or a failure) stays in its tx buffer
    (`bufferFull = true`, `CANtxCount++`) and `CO_CANmodule_process` resends it. That is a software TX queue on
    every platform.
  - The fork's canbus API (`esphome/components/canbus/canbus.h`) is
    `send_data(uint32_t id, bool ext, bool rtr, const uint8_t *data, size_t len)` returning `canbus::Error`, and
    `add_callback([](uint32_t can_id, bool extended_id, bool rtr, const std::vector<uint8_t> &data) {...})`. The
    callbacks run on the main loop.
- **The OD is built at runtime** (`CO_MULTIPLE_OD`).
  - Entries are collected, then turned into a sorted `OD_entry_t[]` plus a blank last element, and an
    `OD_t {size, list}`, before `CO_CANopenInit`.
  - `CO_config_t` (in `CANopen.h`, must outlive the `CO_t`) gets the counts (`CNT_*`, `CNT_ARR_*`) and the
    `ENTRY_H1xxx` pointers (found with `OD_find`). Initialisation installs extensions on those entries, so the list
    must not be rebuilt afterwards.
  - Entries can't be added after init (canopen-stack allowed this until operational); that case is logged.
- **Development tree:** `components_next/canopen` (the YAML key stays `canopen:`). It replaces `components/canopen`
  once it matches it feature for feature. Until then the old component and its tests (`pytest test/`) are not
  touched.
- **Test target:** `test/stm32l4_canopen.yaml` (Zephyr, nucleo_l476rg, MCUboot), built with
  `../esphomeplatformzephyr/venv/bin/esphome` (version 2026.11.0-dev). Upstream build checks with the upstream `dev`
  venv `~/repos/esphome-upstream-venv` (see CLAUDE.md "Testing"): `test/esp32_canopen.yaml` (`-s framework
  arduino` for Arduino) and `test/esp8266_canopen.yaml`.

## CANopenNode facts the code depends on (checked in the source)
- **Init order:**
  1. `CO_new(&config, &heapUsed)`;
  2. `CO_CANinit(co, CANptr, bitRate)`, which calls our `CO_CANmodule_init`;
  3. `CO_CANopenInit(co, NMT=NULL, em=NULL, od, OD_statusBits=NULL, NMTcontrol, firstHBTime_ms,
     SDOserverTimeoutTime_ms, SDOclientTimeoutTime_ms, SDOclientBlockTransfer, nodeId, &errInfo)`;
  4. `CO_CANopenInitPDO(co, co->em, od, nodeId, &errInfo)`, once PDOs exist;
  5. `CO_CANsetNormalMode(co->CANmodule)`.

  On error, `errInfo` holds the OD index that failed.
- **Processing:** `CO_process(co, false, timeDifference_us, NULL)` returns `CO_NMT_reset_cmd_t` (`CO_RESET_NOT`,
  `CO_RESET_COMM`, `CO_RESET_APP`, ...). Then `CO_process_RPDO(co, false, dt, NULL)` and
  `CO_process_TPDO(co, false, dt, NULL)` (no SYNC, so `syncWas = false`). RX callbacks only buffer.
- **`CO_CANopenInit` needs:**
  - `CNT_EM == 1`, or an `em` passed in. `CO_EM_init` needs `ENTRY_H1001` (u8 error register) and, with
    `CO_CONFIG_EM_PRODUCER`, `ENTRY_H1014` (u32, value `0x80`; CANopenNode adds the node id when the CAN-ID is the
    default `0x80`). The EM fifo is allocated only when `CNT_ARR_1003 + 1 >= 2`, so set `CNT_ARR_1003 >= 1`.
    `0x1015` is only needed with `EM_PROD_INHIBIT`; `0x1003` only with `EM_HISTORY`.
  - `CNT_NMT == 1`, with `ENTRY_H1017` (u16 producer heartbeat time in ms).
  - The HB consumer (`CNT_HB_CONS = 1`) needs `CNT_ARR_1016 = n` and `ENTRY_H1016`: an ARRAY of u32
    `(node_id << 16) | timeout_ms`, with sub0 = n.
  - Node id 1..127 (LSS is off).
- **NMT:**
  - `NMTcontrol` flags: `CO_NMT_STARTUP_TO_OPERATIONAL` (0x0100), `CO_NMT_ERR_ON_BUSOFF_HB`, ... .
  - `CO_NMT_sendInternalCommand(co->NMT, CO_NMT_ENTER_OPERATIONAL | CO_NMT_ENTER_PRE_OPERATIONAL | ...)`.
  - `CO_NMT_initCallbackChanged(NMT, void (*)(CO_NMT_internalState_t))` has **no context pointer**.
  - States: `CO_NMT_INITIALIZING` = 0, `PRE_OPERATIONAL` = 127, `OPERATIONAL` = 5, `STOPPED` = 4,
    `UNKNOWN` = -1.
- **HB consumer callbacks** (with `CO_CONFIG_HB_CONS_CALLBACK_MULTI`; it can't be combined with `CALLBACK_CHANGE`),
  each per idx (position in 0x1016 - 1):
  - `CO_HBconsumer_initCallbackTimeout(HBcons, idx, object, void (*)(uint8_t nodeId, uint8_t idx, void *object))`;
  - `initCallbackNmtChanged` (callback also gets the NMT state);
  - `initCallbackHeartbeatStarted`;
  - `initCallbackRemoteReset`.
- **OD types** (`301/CO_ODinterface.h`):
  - `OD_entry_t {uint16_t index; uint8_t subEntriesCount; uint8_t odObjectType; void *odObject;
    OD_extension_t *extension;}`, with `odObjectType` one of `ODT_VAR`, `ODT_ARR`, `ODT_REC`;
  - `OD_obj_var_t {void *dataOrig; OD_attr_t attribute; OD_size_t dataLength;}`;
  - `OD_obj_array_t {uint8_t *dataOrig0; void *dataOrig; OD_attr_t attribute0; OD_attr_t attribute;
    OD_size_t dataElementLength; OD_size_t dataElementSizeof;}`;
  - `OD_obj_record_t {void *dataOrig; uint8_t subIndex; OD_attr_t attribute; OD_size_t dataLength;}`; a RECORD's
    odObject is an array of these, sub0 included;
  - attributes: `ODA_SDO_R/W/RW`, `ODA_TPDO`, `ODA_RPDO`, `ODA_MB` (multi-byte), `ODA_STR`;
  - `OD_find` is a binary search, so the list must be sorted by index.
- **Extensions:**
  - `OD_extension_t {void *object; read(OD_stream_t*, void *buf, OD_size_t count, OD_size_t *countRead);
    write(...); flagsPDO[]}`, installed with `OD_extension_init`. There is one per **index**, so it dispatches on
    `stream->subIndex`.
  - `OD_stream_t` has `dataOrig`, `object`, `dataLength`, `dataOffset`, `attribute`, `index`, `subIndex`.
  - Return `ODR_PARTIAL` to continue and `ODR_OK` when done; errors map to SDO abort codes (`ODR_t`).
  - `OD_readOriginal`/`OD_writeOriginal` do the default copy.
- **PDO:**
  - With `CO_CONFIG_PDO_OD_IO_ACCESS`, mapping uses `OD_getSub(entry, sub, &io, false)`, i.e. the extension IO, so
    our command and slot handlers fire on RPDO.
  - Dummy mappings (index < 0x20) are supported.
  - The TPDO is triggered by `CO_TPDOsendRequest(&co->TPDO[n])` (event-driven, types 0/254/255) and only sent in
    NMT operational.
  - CANopenNode adds the node id to the default COB-IDs only for PDO 1–4, so write full COB-IDs.
  - RPDO 0x1400 subs: 1 COB-ID, 2 transmission type, 5 event timer. TPDO 0x1800 subs: 1, 2, 3 inhibit, 5 event
    timer (6 sync start is only used with SYNC).
  - With `OD_DYNAMIC`, PDO init installs extensions on 0x14xx/0x16xx/0x18xx/0x1Axx.
- **SDO server:**
  - Upload: the first `read()` happens at initiate. If it finishes in one call, the size is known; otherwise
    `stream.dataLength` is used. A read handler can set `stream->dataLength` on the first call (`dataOffset == 0`)
    to indicate the size.
  - `read()` is asked for "free buffer space", so it may be asked for more than is left.
  - A zero-length upload is aborted (`CO_SDO_AB_DEVICE_INCOMPAT`).
  - Block upload refills `blksize * 7` (the client's blksize); a buffer that's too small aborts.
  - Download: data is written to the OD in chunks when the buffer fills; `stream.dataLength` = the size indicated.
  - The first SDO server (0x1200) always uses the default COB-IDs 0x600/0x580 + node id.
- **SDO client:**
  - `CO_SDOclient_setup(SDO_C, cobClientToServer, cobServerToClient, nodeIdOfServer)`;
  - `CO_SDOclientDownloadInitiate(SDO_C, index, sub, sizeIndicated, timeout_ms, blockEnable)`,
    `CO_SDOclientDownloadBufWrite`, then poll `CO_SDOclientDownload(SDO_C, dt_us, abort, bufferPartial, &abortCode,
    &sizeTransferred, NULL)`;
  - `CO_SDOclientUploadInitiate(SDO_C, index, sub, timeout_ms, blockEnable)`, poll `CO_SDOclientUpload(...)`, then
    `CO_SDOclientUploadBufRead`;
  - return values `CO_SDO_RT_ok_communicationEnd` = 0, `waitingResponse` = 1, ... , negative = error;
  - `CO_SDOclientClose`.

## How canopen-stack concepts map to CANopenNode

| esphome-canopen now (canopen-stack, `components/canopen`) | CANopenNode |
|---|---|
| `ObjectDictionary` of `CO_OBJ` + `CO_KEY` + `CO_T*` (`od.cpp`) | runtime `OD_entry_t` list (VAR / ARRAY / RECORD) |
| custom `CO_OBJ_TYPE`: `co_cmd.cpp` (Cmd8/16/32 → `can_cmd_handlers`), `co_storage.cpp` (0x1010/0x1011 "save"/"load"), `fw.cpp` (FwCtrl/FwImage), `logs.cpp` (LogData/Pending/Level) | `OD_extension_t` read/write per index, dispatching per subindex |
| `CONodeProcess` in `on_frame`, `COTmrService`/`COTmrProcess` in `loop()`, `driver_can.cpp` with `current_canopen` | RX callbacks buffer. Process functions run in `loop()` **and right after each received frame** (keeps SDO block latency low) |
| `COTPdoTrigPdo` and `CO_OBJ___A___` (immediate); `dirty_tpdo_mask` handled in `loop()` | `CO_TPDOsendRequest()`. Immediate = request and process now, otherwise dirty mask in `loop()` |
| RPDO config in `uint8_t rpdo_buf[CO_RPDO_N][41]` behind 0x1400/0x1600, persisted with preferences key `canopen_comm_state_v2` (ESP32 only) | the same storage behind RECORD entries (0x1400 sub5 → offset 6), so saved configs keep loading |
| TPDO 0x1800 with `CO_OBJ_DN` (adds the node id), 8 TPDOs, COB-ID `0x180 + 0x100 * n` (+0x80 for n ≥ 4) | full COB-IDs with the node id written at build time. A TPDO with no mapping is marked invalid (bit 31) |
| `CONmtSetMode`, `on_pre_operational` / `on_operational` triggers | `CO_NMT_sendInternalCommand`, `CO_NMT_initCallbackChanged` |
| `CO_THB_PROD` (0x1017), `CO_THB_CONS` (0x1016), `CONmtHbConsEvent` → `on_hb_consumer_event` | the same OD entries, HB consumer callbacks |
| `COCSdo*` via `csdo_recv` / `csdo_send_*` (expedited + segmented, size must be known, no context pointer) | `CO_SDOclient*` (all transfer types, polled) |
| SDO server with `-DCO_SDO_BUF_SEG = sdo_block_transfer_size` | `-DCO_CONFIG_SDO_SRV_BLKSIZE = sdo_block_transfer_size` (buffer stays 900); option dropped for now, see "Block size" |
| `AppEmcyTbl` | `CO_EM` (0x1001, 0x1014) |

Things to watch:
- **Log uploads (`0x3100:1`).**
  - The read handler takes a snapshot of `pending` into `xfer_remaining` when `dataOffset == 0`, sets
    `stream->dataLength`, copies at most `xfer_remaining`, and returns `ODR_PARTIAL` until it's done.
  - The `logs.cpp` rules still apply: never consume more than was announced, and nothing in `logs.cpp` may log.
  - The host only uploads when `0x3100:2` > 0, so the zero-length abort doesn't matter.
- **OTA (`0x3000:4`).**
  - Write handler: `ota->begin(size)` at the first chunk (`dataOffset == 0`; size from `0x3000:2`), then
    `ota->write()`, and `ota->end(md5)` once `size` bytes have arrived.
  - Keep the deferred `ota_finish()`: on some platforms `backend->end()` reboots right away and the SDO block
    transfer must finish first.
  - Keep the `FwImageWrite, N kB (P%)` log every KB (used to measure speed).
- **NMT reset commands.** `CO_RESET_APP` → `App.safe_reboot()`. `CO_RESET_COMM` → log first; re-initialising
  (`CO_delete` + `CO_new`) comes later.
- **Multiple `canopen:` instances on one bus.** The loopback of frames between local instances moves into
  `CO_CANsend` (deliver to the other instances on the same `canbus`). The RX callbacks only buffer, so re-entrancy
  is safe.
- **Platform code from the old `canopen.cpp`:** ESP32 TWAI alerts, bus-off recovery and `get_can_status`. Move it
  later into the core, or better into a `canbus`-level helper.

## Layers (target)
```
entity mapper    (canopen_entity_mapper) ESPHome entities <-> 0x2000+, remote slots 0x4000+, template_entities,
                 remote states (rpdo: {node, entity}, platform: canopen), canopen.<domain>.<action>,
                 OD writer, layout hash   (layout.py, remote.py unchanged)
services         OTA (0x3000), remote logs (0x3100), device API (SDO client queue, NMT, HB, EMCY)
node core        CANopenComponent: OD builder, NodeConfig (heartbeat, HB consumers, SDO clients, PDOs),
                 persistence (preferences), driver glue on ESPHome canbus
CANopenNode      CANopen.c + 301/*  (fork: library.json only; port header in components_next/canopen/canopennode_port)
```
- Only the node core (`canopen.cpp`, `od.cpp`, `driver_can.cpp`) includes CANopenNode headers.
- Mapper and services use a neutral API: `OdType`, `OdHandler` with read/write on an offset stream, and
  `od_add_state()` / `od_add_cmd()` without `CO_*` types.
- `CanopenLogs` (the ring buffer) is separated from its OD glue, so `test/logs` no longer needs `co_core.h` stubs.

## Steps
Each step is a commit on `feature/canopennode` (or its own `feature/...` branch) and builds for
`test/stm32l4_canopen.yaml`.

1. **Minimal node:** NMT, heartbeat producer, heartbeat consumer for `heartbeat_clients`, `on_hb_consumer_event`.
   This proves the library pull-in, the driver, the runtime OD and the process timing. See
   [minimal_canopennode.md](minimal_canopennode.md).
2. **Device API** for profile components (CiA 402 first; functionality of esphome/esphome#21522/#21533, own API):
   SDO client on `CO_SDOclient` (typed/buffer reads and writes, one result path, `cancel_sdo`,
   one channel, local transfers), NMT master, node monitoring on the HB consumer (online/offline/NMT/reset
   events), raw heartbeat/EMCY events, raw frame listeners/send (PDOs of other devices), the `CANopenNode` base
   class, `add_record()`. See [device_api_canopennode.md](device_api_canopennode.md).
   - **2b. `cia402`** on the device API (port of esphome/esphome#21533): process data by PDO (drive TPDOs/RPDO
     remapped over SDO at configuration), SDO only for configuration; plus `SDOSequence` and
     `configure_rpdo`/`configure_tpdo` in `canopen` (own upstream PR). See [cia402.md](cia402.md).
3. **SDO server and identity:** 0x1008/0x1009/0x100A, 0x1018, 0x1200 (`CNT_SDO_SRV = 1`); checked with can2mqtt.
   See [sdo_server_canopennode.md](sdo_server_canopennode.md).
4. **OD extensions (`OdHandler`, dispatch per subindex)** and `0x1010`/`0x1011` store/restore ("save" =
   `0x65766173`, "load" = `0x64616F6C`) with preferences.
5. **Remote logs (`0x3100`):**
   - port the handlers;
   - separate `CanopenLogs` from the OD glue;
   - update `test/test_logs.py` / `test/logs/logs_test.cpp`;
   - check against can2mqtt: block upload with `remote_logs: block_size`, which must be ≤ `sdo_block_transfer_size`.
6. **OTA (`0x3000`, `ota: - platform: canopen`):** domain write handler; check MD5, the deferred end, and the speed
   for block sizes 16 / 32 / 63 on zephyr_can and ESP32.
7. **Entities and TPDOs:**
   - port `entities.cpp` to the neutral API;
   - TPDOs 0x1800/0x1A00 from the mapper, dirty mask / immediate;
   - layout hash `0x3002:1` (FNV-1a over the 0x1A00..0x1A07 mappings, must match `layout_hash()` in `layout.py`);
   - `test/test_layout.py` unchanged.
8. **Commands from the host:** the OD writer (COB-ID `0x500 + node`, data `[node, sub, idx_lo, idx_hi, value...]`,
   written through the OD IO so handlers fire), loopback between local instances. The remote states (RPDO slots
   `0x4000+n`, `rpdo: {node, entity}`, `platform: canopen` entities reading other nodes, `canopen.<domain>.<action>`)
   stay, but in the entity mapper (`canopen_entity_mapper`), not in the `canopen` hub (decision 2026-10-30; replaces
   the 2026-10-29 decision to drop them).
9. **Switch over:**
   - `components_next/canopen` replaces `components/canopen`;
   - remove the canopen-stack code (`co_cmd`, `co_storage`, the old `driver_can`/`od`, the `CO_*` build flags,
     `cg.add_library("canopenstack")`);
   - the remote-state code (`remote.py`, `remote_entities.*`, the `platform: canopen` entity files) moves to
     `canopen_entity_mapper`;
   - update CLAUDE.md and `docs/`;
   - check that `CANOPENNODE_SOURCE` points at the current fork commit (or CANopenNode upstream, if it took the
     `library.json`);
   - ESP32 build (`docker_build.sh`) and a full run against can2mqtt.

## Upstream PRs
Steps are our work order; upstream ESPHome PRs are cut differently. ESPHome prefers small PRs (review limit ~1000
lines of production code, without tests/docs). All PRs target `dev`, so a chained PR carries its parent's commits
until the parent is merged (as #21533 does with #21522): the chain is reviewed serially, so the first PR should go
out early and the next ones are prepared here meanwhile.

Split (revised 2026-10-29: the first version of PR 1, ~1050 production lines, was too big for a first PR):

| PR | Content | Lines |
|---|---|---|
| 1. node + heartbeat monitoring | CANopenNode integration (library, port header trimmed to NMT / HB producer+consumer / EM, driver, runtime OD with VAR/ARRAY), NMT state machine, heartbeat producer, `heartbeat_clients` + `on_node_event`, `is_node_online()` / `get_node_state()` | 845 (measured, incl. comments; tests 175) |
| 2. raw frames and events | `on_heartbeat` / `on_emcy` (+ decoders, `Emergency`), `add_frame_listener()`, `send_frame()` | ~150 |
| 3. SDO client + NMT master | `sdo_client.cpp`, `add_record()` (0x1280), `canopen.sdo_write`, `canopen.nmt`, optional parts | ~450 |
| 4. profile API | `canopen_node.h` (`CANopenNode` base class, hooks, forwarders), `register_node()`; branch `canopen-4-profile`, done ([cia402.md](cia402.md), "PR 4") | 145 (tests ~240) |
| 5. `cia402`, SDO | direct port of #21533 onto the hub (ideally by its author) | ~450 |
| 6. PDO process data | `add_pdo_listener()` / `send_pdo()`, `SDOSequence`, `configure_rpdo/tpdo`; `cia402` `process_data: pdo` | ~450 |

The SDO server and our own entities/OTA/logs (steps 3-8) are a later chain.

Branches, tests and results of PRs 1-3: [upstream_prs_1-3.md](upstream_prs_1-3.md); PR 4 (`canopen-4-profile`,
pushed, not in `components_next` yet): [cia402.md](cia402.md), "PR 4". Each branch is cut from
`components_next/canopen` (the full, hardware-tested tree) in `~/repos/esphome`; later branches stack on it.

Before PR 1:
- **Library source:** no `file://` path. Stock CANopenNode can't be used as is: it has no `library.json`, and the
  Zephyr and native ESP-IDF backends of ESPHome require a manifest (`esphome/platformio/library.py`, only the `host`
  backend accepts manifest-less libraries; plain PlatformIO builds would work). Done (2026-10-29): packaging-only
  fork (branch `esphome`, upstream + `library.json`, pushed), pulled from GitHub by commit; port header in our
  component. Optional: propose the `library.json` to CANopenNode upstream so the fork can go. The port header reaches CANopenNode on ESP32, ESP8266 and Zephyr (see "Port header"). The block-size
  patch (branch `sdo-srv-blksize`) is needed only from step 3 on (drop it, or send it
  upstream).
- **ESP32 / ESP8266 builds** (upstream CI: esp32-idf, esp32-ard, esp8266): all three compile with upstream `dev`
  (`test/esp32_canopen.yaml`, `test/esp8266_canopen.yaml`). `driver_can.cpp` uses `send_data(..., data, len)`,
  which is in `dev` but not in the 2026.9.x releases, so `components_next` needs ESPHome newer than 2026.9 (fine for
  PRs against `dev`; matters only if it ships as an external component before the next release). Still to do: run
  on real ESP32 hardware.
- **API churn first:** the rename and `heartbeat_clients` rebuilt on node monitoring go into PR 1, so PR 2 doesn't
  rewrite code under review.
- **Upstream boilerplate:** `tests/components/canopen/*.yaml`, Python tests, CODEOWNERS, esphome.io docs.
- **#21522** (another author's open `canopen` hub, base of #21533): talk to its author / comment there before
  opening a competing component (open question below).

## Upstream conventions (checked 2026-10-31)
Against upstream `AGENTS.md` and its `embedded-memory` / `automations` skills (Nick Koston's recent memory, string and
callback refactors):
- Done, folded into every PR commit (the branches were rewritten with a tree filter, so each PR passes on its own):
  enum values prefixed with the enum name (`NMTState::NMT_STATE_OPERATIONAL`, `SDOResult::SDO_RESULT_SUCCESS`,
  `OdResult::OD_RESULT_OK`, ...: bare `SUCCESS` / `OK` break LibreTiny, whose SDK defines them as macros), concatenated
  namespaces, static methods without the trailing `_`, a NOLINT block around CANopenNode's C driver API, no
  `volatile` in the port header (main loop only). Docs branches updated too.
- clang-tidy (23.1.4, `script/clang-tidy -e esp8266-arduino-tidy esphome/components/canopen`) is clean. It needs a
  local setup that is **not** in the PRs yet (`test/upstream_tidy_setup.diff`, apply in `~/repos/esphome` with
  `git apply`): CANopenNode and the port directory
  (`file://esphome/components/canopen/canopennode_port`) in `platformio.ini` `lib_deps`, the `CO_CONFIG_*` flags in
  its `build_flags`, and `USE_CANOPEN_*` in `esphome/core/defines.h`. Open for PR 1: upstream's tidy script drops
  include dirs inside the repo (so a plain `-I` to the port header doesn't work), and `esp32-idf-tidy` rejects relative
  `file://` libraries and ignores `build_flags`. Needs a decision before PR 1 goes out.
- Done on `canopen-next` (2026-10-31, one commit, to fold into the PRs when they are cut):
  - `ObjectDictionary` without `std::deque` (512-byte blocks): plain vectors that only grow until `finalize()`;
    entries refer to their objects by position until then, and the pointers CANopenNode gets are taken in
    `finalize()`. `add_*()` / `set_object()` after `finalize()` are ignored. `add_record()` takes an
    `initializer_list` or a pointer and count (no temporary vector); so do `configure_tpdo/rpdo()`.
  - `nmt_state_to_string()`, `node_event_to_string()` and `sdo_result_to_string()` return `const LogString *`
    (`LOG_STR_ARG()` in lambdas); `mark_failed(LOG_STR(...))` where nothing else is logged.
  - `canopen.nmt` registered with `apply_action_builder()` (no action class, a static function calling `send_nmt()`).
  - SDO client without heap use after setup: `write_sdo_data()` no longer copies (**the data must stay valid until the
    callback**, documented); value reads use a 4-byte array, buffer reads one 1 KB buffer allocated on the first one
    and kept.
  - Mapper: no string copies (metadata strings are read from the entity by the handler on each upload: name and
    device class from flash on ESP8266, unit, state class), entities in a `StaticVector` sized by codegen
    (`CANOPEN_ENTITY_MAPPER_ENTITIES`), states in `std::array`, callbacks capturing only the entity pointer,
    `static constexpr`, `ESPHOME_LOG_TAG`, concatenated namespace, Python type hints. stm32l4 flash -2.8 KB.

## Hub for the entity mapper
Target split (2026-10-30): the `canopen` hub (generic CiA 301) goes upstream; everything specific to esphome-canopen
and can2mqtt stays here as the external component `canopen_entity_mapper` (named `esphome_entity_mapper` until
2026-10-31), which refers to a hub with `canopen_id`:
entities (0x2000+16n), TPDO layout and hash (0x3002), remote states (0x4000+n, `rpdo: {node, entity}`,
`platform: canopen`, `canopen.<domain>.<action>`), the OD writer, OTA (0x3000), remote logs (0x3100), and the
ESPHome identity values and version string.

**Milestone (2026-10-30): can2mqtt discovers a node on the new hub.** `test/stm32l4_canopen.yaml` (hub from
`canopen-next`, PRs 1-8) with `test/can2mqtt_test.h`: the ESPHome identity (vendor `0xA59A08F5`, product
`0x6BDFA1D9`, revision 1), one hand-built sensor in the entity layout (0x2001 types, 0x2010 metadata, 0x2011:1 REAL32
state) through `get_od()`, read-only TPDO config records (0x1800-0x1807 / 0x1A00-0x1A07, TPDO 0 on 0x1FF mapped to
0x2011:1) and the TPDO emulated with `send_frame()` every 5 s. can2mqtt ran its whole discovery (identity, strings
as segmented uploads, entity types, metadata, the TPDO config, the initial state) and the sensor shows up in Home
Assistant with live values. Confirms PRs 7/8 against a real client. Since PR 9 the sensor's TPDO is a real CANopenNode
TPDO (`configure_tpdo()`, `request_tpdo()`); what's left for real entities is the mapper building this from YAML.

What the hub still lacks for that (checked against `components_next`, 2026-10-30):
1. **SDO server** (0x1200, `CNT_SDO_SRV = 1`): can2mqtt only talks to nodes through it
   (step 3, [sdo_server_canopennode.md](sdo_server_canopennode.md)). **Done, PR 7**: always on (CiA 301), segmented,
   32-byte buffer.
2. **Device info and identity:** 0x1008/0x1009/0x100A and 0x1018 (vendor, product, revision, serial) as setters or
   YAML options, so the mapper sets the ESPHome vendor/product and the firmware scanner's version string, and
   upstream doesn't carry them. `add_string()` without `ODA_STR` (uploads indicate the size). **Done, PR 7**.
3. **OD API for other components.** **Done, PR 8** (2026-10-30, `canopen-next`): `get_od()` (entries and
   handlers before setup only), `ObjectDictionary::set_object<T>()` with `OdObject` handlers (plain base class with
   default `od_read` / `od_write` on the entry's storage; template trampoline per handler type, no virtual calls),
   `OdStream` (sub, offset, size indication, `read_original` / `write_original`), `OdResult` (= `ODR_t` values),
   `OD_READ` / `OD_WRITE` / `OD_TPDO` / `OD_RPDO`, `write_od()` (synchronous, through the handler), duplicate index =
   setup error. Hardware: local SDO write / read and `write_od()` through a handler, a generated 100-byte domain read
   in PARTIAL chunks. Still missing for PDOs: a way to request a TPDO on change (`OD_requestTPDO`, comes with PR 9).
4. **Own PDOs** (not PR 6, which is about other devices' PDOs): `CO_CONFIG_PDO`, `CNT_TPDO` / `CNT_RPDO` from codegen,
   0x1400/0x1600/0x1800/0x1A00 records with full COB-IDs (CANopenNode adds the node id only for PDO 1-4),
   `CO_CANopenInitPDO`, RPDO/TPDO processing in `process_()`, `request_tpdo(n)` and immediate sends. The TPDO layout,
   the remote-state RPDOs and the raw `rpdo:` linking sit on this. **Done, PR 9** (2026-10-30, `canopen-next`):
   `canopen.enable_pdo()` (`-DCO_CONFIG_PDO=0x402F`: RPDO, TPDO, timers, `OD_IO_ACCESS`, `OD_DYNAMIC`; no
   SYNC), `configure_tpdo(n, cob_id, mappings, event_timer, inhibit, type)` / `configure_rpdo(...)` before setup (the
   hub creates the contiguous records 0..n; unconfigured PDOs and PDOs without mappings are disabled; up to 32 PDOs,
   8 mappings, 64 bits, whole bytes), `request_tpdo(n)` and `request_tpdo(index, sub)`. CANopenNode has one request
   flag per sub-entry: with several TPDOs mapping it, only the first is sent. Unit test with a fake canbus (CANopenNode's
   real PDO code on the host). Hardware: the can2mqtt test sensor as a real TPDO 0 (can2mqtt discovery through the real
   records), node 0x63's TPDO 0 received by RPDO 0 through a handler, no EMCY for the disabled PDOs, heap 5.6 KB with
   8 TPDOs + 1 RPDO. Not covered on hardware: event timer, inhibit time, a remap over SDO. Facts from CANopenNode
   (`301/CO_PDO.c`, `CANopen.c`, checked 2026-10-30):
   - `CO_CANopenInitPDO` takes `ENTRY_H1400/1600/1800/1A00` and steps with `++` (pointer arithmetic, not `OD_find`):
     the N records of each range must be **contiguous** in the OD array (0x1400..0x1400+N-1 all present, no gaps);
   - mapped objects need `OD_TPDO` / `OD_RPDO` and a length >= the mapped bits (byte aligned without
     `CO_CONFIG_PDO_BITWISE_MAPPING`), else the PDO is invalid and an EMCY `CO_EM_PDO_WRONG_MAPPING` is sent; a valid
     COB-ID with 0 mapped objects is invalid too;
   - TPDO triggers: `CO_TPDOsendRequest()`, `OD_requestTPDO(entry, sub)` (only for objects with an extension, the flag
     bits are `OD_extension_t.flagsPDO`: every `set_object()` entry has one), or the event timer. Init sets
     `sendRequest`, so each TPDO goes out once on entering OPERATIONAL. Both process functions only run in
     OPERATIONAL;
   - RPDO data goes through the object's `od_write()` only with `CO_CONFIG_PDO_OD_IO_ACCESS`;
   - with `CO_CONFIG_FLAG_OD_DYNAMIC` PDO init installs CANopenNode's own extensions on 0x14xx-0x1Axx (live changes
     over SDO): no `set_object()` there; store/restore (PR 10) reads the records' data;
   - COB-IDs: any valid 11-bit ID; only a stored predefined base (0x180, ...) gets the node id added, for PDO 1-4
     (`CO_xPDO_DEFAULT_CANID_COUNT = 4`). The old component's TPDO 4-7 on 0x200/0x300/0x400/0x500 + node (the RPDO
     defaults) work as full COB-IDs; SDO writes to them are allowed (not restricted IDs).
5. **Store/restore** (0x1010/0x1011 "save"/"load" into preferences): the host changes the RPDO config and it must
   survive a reboot. An upstream hub uses its own preferences key, so saved configs (`canopen_comm_state_v2`) are lost
   unless the mapper migrates them (to decide).
6. **Own NMT state.** **Done, PR 8**: `get_nmt_state()`, `set_nmt_state()` (no NMT master needed),
   `on_nmt_state_change`, `nmt_startup: operational | pre_operational` (default operational). Tracked in `process_()`
   (CANopenNode's change callback has no context pointer).
7. **SDO block size** (`sdo_block_transfer_size`, see "Block size"): needed for OTA and log uploads. **Done, PR 12**
   (2026-10-31, `canopen-next`): hub option `sdo_block_size` (1..127, unset = segmented only, 32-byte buffer) and
   codegen helper `enable_sdo_block(block_size)`: `CO_CONFIG_SDO_SRV` segmented | block, buffer 900 (CANopenNode's
   minimum for block), `CO_CONFIG_SDO_SRV_BLKSIZE` (caps the announced download block size; needs the fork's patch,
   upstream CANopenNode ignores it and announces up to 127), CRC16. Uploads use the client's block size (CANopenNode
   doesn't cap them: remote logs' `block_size` is advertised in 0x3100:7 as before). Unit test: a hand-played block
   download client on the fake canbus (announced size, 300 bytes in 3 blocks into a domain handler, CRC; a wrong CRC
   aborts with 0x05040004). Zephyr: the flags reach `CO_SDOserver.c` (compile_commands). Not on hardware yet: comes
   with OTA (can2mqtt uses block download only for firmware). Still open: YAML sizes for the server buffer and the
   client FIFO.
8. **Loopback between local nodes** in the driver (`CO_CANsend` and `send_frame()` deliver to the other `canopen:`
   nodes on the same canbus; was step 8).
9. **Codegen hooks:** `enable_sdo_server()`, `enable_pdo(tpdo=N, rpdo=M)`, and a defined point where other
   components add OD entries before the hub's `setup()` (from codegen / constructors, or a setup priority above
   `PROCESSOR`).

Not needed in the hub: raw frames and monitoring exist already (OD writer = frame listener, remote commands =
`send_frame()` / `write_sdo()`, `monitor_node()`); ESP32 TWAI alerts / bus-off recovery / `get_can_status` belong in
`esp32_can` / `canbus`; `CO_RESET_COMM` re-init is simpler if extensions and PDO configs are kept in the hub's own
lists.

Upstream PRs for it (estimates, not measured; independent of PRs 4-6, could go before them):

| PR | Content | Lines |
|---|---|---|
| 7 | SDO server, device info / identity, `add_string()`; done, `canopen-next` ([sdo_server_canopennode.md](sdo_server_canopennode.md)) | ~100 (tests ~85) |
| 8 | public OD API, `OdObject` handlers, `write_od()`, own NMT state and `nmt_startup`; done, `canopen-next` | ~240 (tests ~115) |
| 9 | own PDOs (records, init, processing, `request_tpdo()`); done, `canopen-next` | ~170 (tests ~165) |
| 10 | 0x1010/0x1011 store/restore | ~200 |
| 11 | loopback between local nodes | ~100 |
| 12 | SDO server block transfer, `sdo_block_size`; done, `canopen-next` (the size cap needs the fork's patch upstream) | ~40 (tests ~150) |

PRs 7-9 are the minimum for the mapper against can2mqtt; 10-12 for full parity.

All of them are developed on one branch, `canopen-next` in `~/repos/esphome` (from `canopen-4-profile`, one commit per
PR), and cut into stacked PR branches when they go out. `test/stm32l4_canopen.yaml` builds `canopen` from
`../../esphome`, i.e. whatever branch is checked out there.

User docs: `~/repos/esphome-docs` branch `canopen-next` (from `canopen`, the PRs 1-3 page; `004611e`, pushed to
mrk-its/esphome-docs, no PR) adds to `components/canopen.mdx` what PRs 7-9 add for YAML users: identity and device
information, the SDO server, an object dictionary table, `nmt_startup`, `on_nmt_state_change`,
`get_nmt_state()` / `set_nmt_state()`. The C++ APIs for other components (PR 4 profile API, PR 8 OD handlers,
PR 9 PDO configuration) aren't documented there. Split per PR like the code when the PRs go out.

Entity commands (0x2002+16N, 2026-10-31): one write-only, RPDO-mappable sub per state with the same size, storage in
the entity, run by the entity's `od_write()` when the write is complete (SDO, RPDO, `write_od()`). Semantics as the
old component: sensor / number / binary sensor publish the value (a number is *published*, not `make_call()`, so its
`set_action` doesn't run; to fix, see the open points), switch on / off, light state / brightness / color temperature, cover stop / open / close /
position / tilt, alarm disarm / arm / trigger (127); a scaled NaN (255 / 65535) is ignored. Tested on the nucleo with
local SDO writes (`test/stm32l4_canopen.yaml`). CANopenNode quirk: the *local* SDO client reads a write-only entry
anyway (`CO_SDOclientUpload()` sets the write-only abort but then reads and overwrites the result); the SDO server
checks the access correctly.

Mapper tests: `test/test_entity_mapper.py` (in `pytest test/`). It runs codegen and validation through the upstream
venv's CLI (`esphome compile --only-generate`, `esphome config`) with the hub from `~/repos/esphome` as a local external
component (`$ESPHOME_UPSTREAM_VENV`, `$ESPHOME_CANOPEN_HUB`; skipped without them). It checks: one entity of every
domain to the `add_*()` calls in index order; the 8 `configure_tpdo()` calls (COB-IDs, mappings, the empty ones
disabled); the layout hash; the identity after the hub's default; the version string; the entity count define; and
the validation errors (duplicate index, unsupported domain, TPDO over 8 bytes, caps, size, ranges). It also checks
that `layout.py` is identical to the old component's copy. ESP8266 build check: `test/esp8266_mapper.yaml`.

Open points for `canopen_entity_mapper` itself (not in the steps yet):
- number commands through `make_call()` (own step, after the port): today a number command only publishes the value
  (old component's behaviour), so a template number's `set_action`, `min_value` / `max_value` / step checks and
  restore don't run. `number->make_call().set_value(v).perform()` instead; the state then comes back through the
  normal state callback (optimistic or set by the action). Changes behaviour for existing nodes: note it in the
  release notes / `configuration.md` and update `OBJECT_DICTIONARY.md` ("sets the number" instead of "publishes").
- OD writer **ported as is** (2026-10-31): `od_writer: true` (default; the old `pdo_od_writer`) in the mapper, a hub
  frame listener on 0x500..0x57F, byte 0 = target, written with `write_od()` (handlers run, so the entity commands
  work through it); `send_od_write(node, index, sub, value, size)` sends `0x500 + target` (our own node is written
  directly, no frame). TPDO 7 is rejected for entities while it's on (`od_writer: false` frees it). The node id comes
  from codegen (`set_node_id()`, the hub has no getter). Tested on the nucleo: injected frames (own id, other id,
  bad index, short) and a real frame to dimmer4 (node 0x37, old component) turning light5 off, state read back over
  SDO.
- **Remote states ported** (2026-10-31): `remote_nodes` on the mapper (`remote.py` reads both formats: old
  `canopen: entities` and hub `canopen:` + `canopen_entity_mapper: entities`, plus the target's OD writer flag),
  `rpdo: {node, entity, state, cmd}` on entities, remote state slots 0x4000+n (`RemoteSlot`, an `OdObject`, in a
  fixed `std::array` so listeners can attach before the slot sizes are set), `binary_sensor` / `sensor` /
  `text_sensor` **`platform: canopen_entity_mapper`** (was `platform: canopen`: a platform module must live in the
  component's directory, and `canopen` is now the upstream hub), and the `canopen.<domain>.<action>` actions (names
  kept; `RemoteCommandAction` with at most 3 writes, sent with `send_od_write()`; an action to a node with the OD
  writer off is rejected). RPDOs through the hub's `configure_rpdo()`, one per remote (node, TPDO), mapping the
  remote TPDO's **full length** (CANopenNode reports a longer frame with an EMCY): gaps and the tail are single
  dummy entries (index 0x0005, sub 0, any length); the plan (`remote.rpdo_plan()`, ESPHome-free) rejects overlaps
  (a state in a slot and in an `rpdo:`), more than 8 entries. No 4-RPDO limit any more (hub: 32). Tests:
  `test/test_mapper_remote.py`, codegen / errors in `test/test_entity_mapper.py`; nucleo against the real dimmer4
  (node 0x37, old component): slots follow light5, `canopen.light.turn_off` turns it off, `test_switch` follows
  light4 by `rpdo:`, "Biuro toggle" button in HA. Not ported: `canopen.entity_cmd`, `template_entities` (open points
  below).
- **Remote logs ported** (2026-10-31): `canopen_entity_mapper: remote_logs:` (was `canopen: remote_logs:`; same
  options: `buffer_size`, `level`, `timeout`, `block_size`; an empty block is `remote_logs: {}`), OD 0x3100 unchanged
  (can2mqtt unchanged). Split: `log_buffer.*` (ring buffer, level, keepalive, timeout; no CANopen / ESPHome code but
  the log level macros and `ESPHOME_strcmp_P`) and `remote_logs.*` (`RemoteLogs`, a component and `OdObject`: the
  record, the handler, the logger callback registered from codegen before `App.setup()`, `loop()` for the timeout,
  enabled only while logging is on with a timeout). Log Data: the snapshot at the first read (`offset == 0`, the size
  indication), each read copies at most the rest, PARTIAL until done; an empty buffer gives a zero-length upload,
  which CANopenNode aborts. Changes: the `block_size <= sdo_block_transfer_size` check is gone (the hub's
  `sdo_block_size` caps downloads only, uploads use the client's size and the 900-byte buffer takes 127 segments);
  without `sdo_block_size` on the hub, block transfer is enabled with the mapper's `DEFAULT_SDO_BLOCK_SIZE` (16, shared
  with OTA); the canbus / canopen tag filter uses `ESPHOME_strcmp_P` (the old `strcmp` read flash tags on ESP8266).
  Tests: `test/logs/log_buffer_test.cpp` (in `test/test_logs.py`, next to the old one), `test_codegen_remote_logs` /
  `test_invalid_remote_logs`. Builds: `test/esp8266_mapper.yaml`, `test/stm32l4_canopen.yaml` (`block_size: 16`,
  tx queue 32). Nucleo against can2mqtt (2026-10-31, `test/stm32l4_biuro_switch.yaml`, level DEBUG, block size 16):
  subscribed with AUTO, lines arrive in can2mqtt's log. can2mqtt logged node lines at their own level, so `[D]` lines
  were hidden at its `--log-level INFO`: can2mqtt now logs lines below INFO at INFO. User docs (`docs/remote-logs.md`)
  with the migration.
- **OTA ported** (2026-10-31): `ota: - platform: canopen_entity_mapper` (`canopen_id`), OD 0x3000 as before (can2mqtt
  unchanged), one `OdObject` handler: the first chunk of :4 starts the backend (size from :2), the last one (a domain's
  stream size is 0 until the last chunk, then the total: CANopenNode doesn't pass the client's indicated size) sets
  the MD5; backend `end()` and the reboot deferred to `loop()` (1 s each, the loop disabled otherwise). OTA state
  listeners (`on_begin` / `on_progress` / ...), `reboot: false`. Needs SDO block transfer: the hub's
  `sdo_block_size`, else 16. Compression (miniz) off on stm32 / zephyr, `miniz.c` filtered out there. Nucleo
  (`test/stm32l4_biuro_switch.yaml`, block size 32, rx queue 64): 114 kB from can2mqtt in ~50 s (~2.3 KB/s), MD5 OK,
  MCUboot swap, new version running. Image confirmation (2026-10-31, nucleo, two CAN OTAs from can2mqtt): the image boots as a test upgrade
  (MCUboot `Swap type: revert` armed), `safe_mode` confirms it 60 s after boot ("Boot seems successful"), then it
  survives a reset (`Swap type: none`); an image reset before the confirmation is reverted by MCUboot, the previous
  one boots again. With `safe_mode: disabled: true` nothing confirms, so the OTA platform warns on Zephyr. Open:
  speed with bigger blocks / a bigger server buffer.
- **MPDO for the OD writer (own step, changes the wire protocol, needs can2mqtt and all nodes updated):** today
  `0x500 + target` is also the target's TPDO 7 (`0x500 + node`), so a TPDO 7 frame whose first byte equals another
  node's id is applied there as an OD write, and the ID has several producers. Proposal: the CiA 301 MPDO format
  (destination address mode: byte 0 = `0x80 | target`, 0 = all; index LE; subindex; 4 data bytes) on the
  **sender's** TPDO 7 (`0x500 + sender`), TPDO 7 reserved in `layout.py`; can2mqtt needs a node id of its own if
  it sends them. Alternatives: extended 29-bit IDs, or `0x680 + sender` (only senders up to 0x5F). Needs a
  migration (accept both formats for a while).
- **Raw RPDO linking ported** (2026-10-31): `rpdo: {node_id, tpdo, offset, cmd, tpdo_size}` on mapper entities, one
  byte at `offset` of the remote TPDO into command `cmd`, a source of the same `remote.rpdo_plan()` as the named form
  (grouped with named entries and slots of the same node / TPDO, overlaps rejected). New: optional `tpdo_size` (1..8),
  the remote TPDO's length, which the RPDO then maps in full; unset, the RPDO ends at the last mapped byte and a longer
  frame is still processed, but CANopenNode sets `CO_EM_RPDO_WRONG_LENGTH` (one EMCY). A named entry of the same TPDO
  gives the size too; different sizes are an error. Stricter than the old component: the command must be 1 byte
  (was any size), `node_id` 1..127, `tpdo` 0..7, `offset` 0..7. Document it with the user migration
  (`configuration.md`, "RPDO schema"). Tests: `test_rpdo_plan_raw*` in `test/test_mapper_remote.py`,
  `test_codegen_raw_rpdo` / `test_invalid_raw_rpdo` in `test/test_entity_mapper.py`. Not on hardware yet.
- `template_entities` (custom OD entries with typed command triggers, states, metadata);
- `canopen.entity_cmd` (a command to another node's entity, on `write_sdo` or the OD writer frame);
- `csdo:` and its lambda API (`setup_csdo`, `csdo_recv`, `csdo_send_*`): map onto the hub's SDO client; custom
  tx/rx IDs need the pinned channels (device_api_canopennode.md, "Open points");
- user migration: YAML changes from one `canopen:` block to `canopen:` + `canopen_entity_mapper:`; update
  `configuration.md` / `linking-nodes.md`;
- until the hub is merged, where the mapper gets the hub from (the esphome fork branch as an external component, or
  `components_next` here).

## Verification for each step
- `../esphomeplatformzephyr/venv/bin/esphome config test/stm32l4_canopen.yaml`, then `compile`.
- Check that CANopenNode sources and our `.cpp` get the same `-DCO_CONFIG_*` flags. The build dir is
  `.esphome/build/<esphome name>/`; look in `compile_commands.json` under the west build. If they differ, the struct
  layouts and buffer sizes differ between the stack and the app.
- `pytest test/` (layout, remote, logs).
- On hardware against can2mqtt: discovery, states/commands, OTA, logs, heartbeat consumer events. Debugging notes
  for CAN OTA are in CLAUDE.md ("Debugging CAN OTA").

## Open questions
- Should `CO_RESET_COMM` really re-initialise the stack (`CO_delete` + `CO_new`), or is ignoring it enough?
- Start pre-operational and wait for `on_pre_operational`, as today (where without that trigger
  `set_pre_operational_mode()` goes straight to operational), or use `CO_NMT_STARTUP_TO_OPERATIONAL` when no such
  trigger is configured?
- RAM on STM32: the SDO server buffer needs ≥ 900 bytes with block transfer enabled, and the SDO client FIFO is
  1000 bytes by default. Size both from YAML.
- EDS export for third-party masters (generated from the runtime OD), later.
- How to handle esphome/esphome#21522 (and #21533 on top of it): coordinate with its author, propose this
  component as its replacement, or build on it?

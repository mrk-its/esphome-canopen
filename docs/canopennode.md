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
  - all on top of upstream master `9b8beed` (2026-07-10); `esphome` is pushed (2026-10-29), the others are local:
    - branch `esphome`, commit `f38ec8a` "Add library.json": packaging only, no stack changes. `"build": {"srcDir":
      ".", "includeDir": ".", "srcFilter": ["+<CANopen.c>", "+<301/*.c>"]}`. This is what the build uses, and the
      `library.json` could be proposed to CANopenNode upstream as is;
    - branch `sdo-srv-blksize`, commit `d1d354a`: the `CO_CONFIG_SDO_SRV_BLKSIZE` patch alone (below), needed from
      step 3 on (or not at all, see "Block size");
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
  - Done 2026-10-29: the default is the git URL (checked: Zephyr, ESP32 esp-idf, ESP8266 clone `f38ec8a` and build).
- **Block size: `sdo_block_transfer_size` dropped for now (2026-10-29), add it back later.** It needs
  `CO_CONFIG_SDO_SRV_BLKSIZE` (the fork patch, option "D", branch `sdo-srv-blksize`, not in the `esphome` branch the
  build uses), which caps the block-download size the SDO server announces. Without it CANopenNode always announces
  127. To add it back: get the patch into CANopenNode upstream (or merge `sdo-srv-blksize` into `esphome`), then
  restore the option (1..127, default 63) → `-DCO_CONFIG_SDO_SRV_BLKSIZE`. Needed at the latest for CAN OTA
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
entity mapper    ESPHome entities <-> 0x2000+, remote slots 0x4000+, template_entities,
                 canopen.<domain>.<action>, OD writer, layout hash   (layout.py, remote.py unchanged)
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
   written through the OD IO so handlers fire), loopback between local instances. The old remote states (RPDO
   slots `0x4000+n`, `platform: canopen` entities reading other nodes, `canopen.<domain>.<action>`) are **dropped**
   from the new component (decision 2026-10-29).
9. **Switch over:**
   - `components_next/canopen` replaces `components/canopen`;
   - remove the canopen-stack code (`co_cmd`, `co_storage`, the old `driver_can`/`od`, the `CO_*` build flags,
     `cg.add_library("canopenstack")`) and the remote-state code (`remote.py`, `remote_entities.*`, the
     `platform: canopen` entity files);
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
| 4. profile API | `canopen_node.h`, `add_on_node_event(node, cb)`, `register_node()`, `SDOSequence`, `configure_rpdo/tpdo` | ~350 |
| 5. `cia402` | the drive profile | ~500 |

The SDO server and our own entities/OTA/logs (steps 3-8) are a later chain.

In `~/repos/esphome` (branches from `upstream/dev`; `canopen-1-node` and `canopen-2-frames` pushed to `origin`
= mrk-its/esphome, no PRs opened):
- (the `canopen` reference branch with the full component is deleted, 2026-10-29: `components_next` is the full
  tree, each PR branch is cut from it)
- `canopen-1-node`: PR 1, one commit, cut from `components_next` by removing the later parts (raw events, frame
  listeners, `send_frame()`, `add_record()`, `add_on_node_event(node, cb)`, the port header's SDO/PDO/CRC/FIFO
  config). Later PR branches stack on it and add those parts back.
- `canopen-2-frames`: PR 2 on top of `canopen-1-node` (one commit, +136 component lines, +245 with tests): raw
  heartbeat/EMCY triggers, frame listeners, `send_frame()`, and a gtest unit test of the decoders
  (`tests/components/canopen/frames_test.cpp`, run with `script/cpp_unit_test.py canopen`).
  `tests/components/canopen/__init__.py` makes the unit test build work: the runner suppresses `to_code` and gives
  MULTI_CONF components an empty list, so the override sets `multi_conf = False` and its `to_code` only calls
  `add_canopennode_library()` (CANopenNode + port header, factored out of `to_code` in PR 1). `add_record()`,
  `add_on_node_event(node, cb)` and the SDO/PDO port config wait for PR 3/4 (no user before).
- `canopen-3-sdo`: PR 3 on top of PR 2 (one commit, +631 component lines, +107 tests): SDO client
  (`sdo_client.cpp`, `automation.h`), NMT master, `canopen.sdo_write` / `canopen.nmt`, `response_timeout`,
  `add_record()`. Tests: action codegen and value/size validation in `tests/component_tests/canopen`, the actions in
  `tests/components/canopen/common.yaml`, gtest `sdo_reply_test.cpp` (`SDOReply::as<T>()`). Hardware: `test/sdo_test.h`
  against node 0x61 (24/24 pass, see device_api_canopennode.md). Not pushed.
  Checked: `script/ci-custom.py`, ruff, clang-format 23.1.3 and the pre-commit hooks pass;
  `tests/component_tests/canopen` (9 tests); `script/test_build_components.py -c canopen --isolate canopen` compiles
  esp32-idf, esp32-ard, esp8266-ard (mcp2515) and rp2040-ard (mcp2515) with no warnings; the trimmed component runs
  on the nucleo (node 0x01 online/NMT changed). Also in the PR: `script/ci-custom.py` exempts `CO_driver_target.h`
  from the namespace and `#define` lints (a C header, as `ota_esphome_inflate.h` / `esp_now_hosted_rpc.h`), and
  `CODEOWNERS`.
- Running upstream scripts: `PATH=~/repos/esphome-upstream-venv/bin:$PATH PYTHONPATH=~/repos/esphome`, so the
  repo's `esphome` is used (the venv has `requirements_test.txt` and `clang-format==23.1.3`).
- The esphome repo gitignores `CMakeLists.txt`, so the ESP32 IDF port component is generated in the build dir by
  codegen (header copy + `CMakeLists.txt`).

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

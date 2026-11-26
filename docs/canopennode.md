# Porting esphome-canopen to CANopenNode

Status (2026-10-31): the hub (upstream `canopen` component, CANopenNode) is done for the entity mapper (hub PRs 1-12
on `canopen-next` in `~/repos/esphome`); `canopen_entities` is ported (this repo, branch `canopen_entities`, renamed from `feature/entity-mapper` 2026-11-01;
see "Hub for the entity mapper"). **Next:** cut the upstream PRs again from the refactored hub, see "Upstream PRs",
"New split" (branch `canopen-full`, refactor committed as `e9f738ebf7`).

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
Where to continue (2026-11-24):
- **Upstream split:** "Upstream PRs" -> "SDO client only first" (the current split) and its "PR 1 as cut". PR 1 (the
  SDO client, no node; `canopen-pr1-sdo-client`) is **merged upstream**: esphome/esphome#22410, squash `206bb68485`
  on `dev` (2026-11-24), identical to the cut's tip. The docs PR esphome/esphome.io#8059 is merged too (2026-11-25, squash `d3f9241e3` on `next`). bdraco's 2026-11-23 round is ported to
  `canopen-full` (`534c38bb71`). Then `canopen-full` was rebuilt on `upstream/dev` (`206bb68485`) as one commit,
  `a055ba97aa` "[canopen] PRs 2-8 on top of PR 1 (to be split)": `canopen-full`'s files for the 59 paths it changes
  (the two PR 1 test configs it renamed deleted), dev's for the rest; 109 C++ + 40 Python tests pass. The old history
  is `canopen-full-backup2` (pushed). `git diff upstream/dev canopen-full` is now exactly PRs 2-8. PR 2 is cut on
  `canopen-pr2-frames` (`20f842f1b3`, pushed; "PR 2 as cut"), upstream PR esphome/esphome#23266 (2026-11-24); its docs are on esphome-docs branch `canopen-pr2-frames` (rebased on `next` after #8059 merged, `d64270a78`; the pre-rebase branch is `backup/canopen-pr2-frames-pre-rebase`), docs PR esphome/esphome.io#8243 (2026-11-25, merged the same day). PR 3's docs: esphome/esphome.io#8254. `~/repos/esphome-docs` `canopen-full` is not
  rebuilt yet (#8059 is merged, so it can be). "New split" holds the refactor and the cutting steps; the old
  `canopen-pr1-node` (the node core) is superseded. PR 2 is **merged upstream** (2026-11-25, squash `29f8ea8e39`,
  identical to the cut head `79dc2e5ede`, with bdraco's 5 review commits, ported to `canopen-full`). `canopen-full`
  was rebuilt again on `upstream/dev` as `f0e18d854d` "[canopen] PRs 3-8 on top of PR 2 (to be split)" (the history
  before is `canopen-full-backup3`, pushed), then `72308fbd7e` (fixes from the PR 3 cut). PR 3 is cut on
  `canopen-pr3-node` and open upstream: esphome/esphome#23299 ("PR 3 as cut").
- **Mapper:** open points at the end of "Hub for the entity mapper"; sizes vs the old component in "Size comparison".
- **Repo layout** (2026-11-01, `canopen_entities`): `components/` holds only `canopen_entities` (moved from
  `components_next`); the old component (`components/canopen`, canopen-stack) and `components_next/canopen` (a stale
  copy of the hub) are removed, with the tests and configs only they used (`test_od.py`, `test_frames.py` (the hub has
  both tests), `test_remote.py`, `logs_test.cpp`, `esp32_canopen.yaml` / `esp8266_canopen.yaml`, the Docker build,
  `examples/`, CI's build job). The old component stays on `main`; tag `main` (e.g. `v0.4.0`, the last tag `v0.3.1`
  is from 2023) before this branch merges, so its users can pin it. User docs rewritten for the hub +
  `canopen_entities` (2026-11-01, `docs/migration.md` for old -> new), with this repo at `@canopen_entities`:
  change that ref when the branch is merged.
- **ESP32 hardware** (2026-11-01): `~/repos/zrodlana/esphome/node-c3-t2.yaml` (ESP32-C3, ESP-IDF, `esp32_can`,
  node 0x32, built with the esphomeplatformzephyr venv) migrated to `canopen` + `canopen_entities`
  (`common/common_canopen_entities.yaml`: no CAN status counters, no "Disable OTA reboot" switch). Updated by CAN OTA
  from the old component through can2mqtt, then again with `remote_logs` (block size 16, `tx_queue_len: 128`):
  entities in HA, toggles of lights on `dimmer1` / `dimmer4` (old component, old-format YAML read by `remote.py`),
  remote logs in can2mqtt, all working. The other `zrodlana` nodes still use `common/common.yaml` (old component, via
  `../../esphome-canopen/components`): build them from `main`.
- **Hub copy in this repo** (2026-11-01): `components/canopen` is a generated copy of `canopen-full`
  (`scripts/sync_hub.py`, `SYNCED_FROM`, checked by `test/test_hub_copy.py`), so users need one `external_components`
  source (a `github://` source of the ESPHome fork is a 42 MB shallow clone for the 110 KB hub). The hub is still
  developed in `~/repos/esphome`; re-sync after each hub commit. Delete the copy once the full hub is released.

The bullets below are the state of step 1 (2026-10-29), kept for the CANopenNode fork and port header details.
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
    - branch `esphome`, commit `30a669c` (2026-11-01: + the port header `CO_driver_target.h` in the library root, see
      "Port header"; before that `9ff367b`, 2026-10-30: reset to `library-json` + cherry-pick of `67ad4ba`,
      force-pushed; was `f38ec8a`, library.json only): what every build pins (`CANOPENNODE_SOURCE` in the hub;
      pinned to the commit, not the branch);
    - branch `esphome-old-dce5a01`: the old all-in-one commit (library.json + port header + block-size patch), kept
      for reference.
- **Port header** `CO_driver_target.h`: in the fork's root (the library's `includeDir`) since 2026-11-01, commit
  `30a669c`. Before that it was in the hub (`canopennode_port/`, moved out of the fork 2026-10-29), added by codegen
  per backend (a generated IDF component on ESP32, a header-only library on Zephyr, `-I` on PlatformIO), because
  project `-I` flags don't reach converted libraries. Moved back because `301/CO_driver.h` includes it
  unconditionally and upstream's `script/clang-tidy` drops include dirs inside the ESPHome repo (only `-I <root>` and
  dirs under `.pio/` survive), so tidy couldn't find it there; esp32-idf-tidy also rejects `file://` libraries. In the
  library it is found on every backend with no codegen at all (`add_canopennode_library()` only adds the library).
  Changing the header now means a fork commit + a `CANOPENNODE_SOURCE` bump.

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
  - `canopennode_path: <absolute path>` (a local checkout instead, `file://`) was dropped on 2026-11-25 (Copilot on
    #23299: per hub although firmware-wide, relative paths against the working directory; nothing used it).
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
- **Development tree:** first `components_next/canopen` here, then the hub in `~/repos/esphome` (upstream PR
  branches, now `canopen-full`) plus `canopen_entities` here. Since 2026-11-01 `canopen_entities` is in `components/`
  and the old component is gone from `canopen_entities` (see "Current state").
- **Test target:** `test/stm32l4_canopen.yaml` (Zephyr, nucleo_l476rg, MCUboot), built with
  `../esphomeplatformzephyr/venv/bin/esphome` (version 2026.11.0-dev). Upstream build check with the upstream `dev`
  venv `~/repos/esphome-upstream-venv` (see CLAUDE.md "Testing"): `test/esp8266_mapper.yaml`; the hub's own builds
  are upstream's `tests/components/canopen/test.*.yaml`.

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
- **Multiple `canopen:` instances on one bus.** The loopback of frames between local instances is in the hub's
  `transmit()` (PR 11): the other instances only receive (buffer), their processing runs in their own `loop()`.
- **Platform code from the old `canopen.cpp`:** ESP32 TWAI alerts, bus-off recovery and `get_can_status`. Move it
  later into the core, or better into a `canbus`-level helper.

## Layers (target)
```
entity mapper    (canopen_entities) ESPHome entities <-> 0x2000+, remote slots 0x4000+, template_entities,
                 remote states (rpdo: {node, entity}, platform: canopen_entities), canopen_entities.<domain>.<action>,
                 OD writer, layout hash   (layout.py, remote.py unchanged)
services         OTA (0x3000), remote logs (0x3100), device API (SDO client queue, NMT, HB, EMCY)
node core        CANopenComponent: OD builder, NodeConfig (heartbeat, HB consumers, SDO clients, PDOs),
                 persistence (preferences), driver glue on ESPHome canbus
CANopenNode      CANopen.c + 301/*  (fork: library.json + SDO block size patch + port header CO_driver_target.h)
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
   stay, but in the entity mapper (`canopen_entities`), not in the `canopen` hub (decision 2026-10-30; replaces
   the 2026-10-29 decision to drop them).
9. **Switch over** (2026-11-01, `canopen_entities`: the old component removed, `canopen_entities` in
   `components/`; the hub stays in `~/repos/esphome`, so the first bullet became "the hub replaces it"):
   - `components_next/canopen` replaces `components/canopen`;
   - remove the canopen-stack code (`co_cmd`, `co_storage`, the old `driver_can`/`od`, the `CO_*` build flags,
     `cg.add_library("canopenstack")`);
   - the remote-state code (`remote.py`, `remote_entities.*`, the `platform: canopen` entity files) moves to
     `canopen_entities`;
   - update CLAUDE.md and `docs/`;
   - check that `CANOPENNODE_SOURCE` points at the current fork commit (or CANopenNode upstream, if it took the
     `library.json`);
   - ESP32 build (`docker_build.sh`) and a full run against can2mqtt.

## Upstream PRs
Steps are our work order; upstream ESPHome PRs are cut differently. ESPHome prefers small PRs (review limit ~1000
lines of production code, without tests/docs). All PRs target `dev`, so a chained PR carries its parent's commits
until the parent is merged (as #21533 does with #21522): the chain is reviewed serially, so the first PR should go
out early and the next ones are prepared here meanwhile.

### New split (2026-10-31, branch `canopen-full`) — superseded by "SDO client only first" below
(The refactor, the file layout and the cutting steps still apply; the PR numbers, the line count table and the
`defines.h` table are of this split. `scripts/pr_split_lines.py` now prints the 2026-11-05 split.)
**Why.** The old split (tables further down) followed the order the work was done in: every PR edited the same
functions (`setup()` mixed every feature, 7 `#ifdef` blocks), so a fix in one PR had to be rebased through all later
ones. None of the old branches (`canopen-1-node` .. `canopen-4-profile`, deleted 2026-11-01, locally and on mrk-its/esphome) was opened as an upstream PR
(only pushed to mrk-its/esphome), so the split is redone from the finished tree: refactor the hub by feature first, so that each PR
mostly *adds files and blocks*, then cut the PR branches from it.

**Where.** `~/repos/esphome`, branch `canopen-full`, created from `canopen-next` (`21ab50897e`, hub PRs 1-12 in 17
commits on `upstream/dev` `b155e4ba0b`), plus the refactor as one commit, `e9f738ebf7` "[canopen] Split the hub by
feature" (2026-10-31, pushed to mrk-its/esphome). `canopen-next` stays as it was;
`canopen-full` replaces `canopen-next` as the hub branch. The test configs here (`test/stm32l4_*.yaml`,
`test/esp8266_mapper.yaml`) use the hub copy in `components/canopen`, synced from `canopen-full`
(`scripts/sync_hub.py`; `--worktree` for uncommitted hub changes).

**Stacked on PR 1 (2026-11-01).** `canopen-full` was rebased on `upstream/dev` `0548ad7a72` (clean, canopen files
unchanged), then rewritten as `canopen-pr1-node` + one commit "[canopen] PRs 2-6 on top of PR 1 (to be split)"
(`30cc4937e3`, same tree as before). The previous history (the 17 + refactor + tidy commits, rebased) is
`canopen-full-backup` (`68cbae6c54`); `e9f738ebf7` and `62bd875` below name commits of the history before that. The
same in `~/repos/esphome-docs`: `canopen-full` = `canopen-pr1-node` + "Document PRs 2-6 (to be split)"
(`0a03b105e`), backup `canopen-full-backup` (`cbb190d07`). All pushed (force-push for `canopen-full`). Cutting
PR N from here: split PR N's part out of the "PRs 2-6" commit into its own commit below it (branch
`canopen-prN-...` at that commit), rest stays on top; review fixes go into the PR commits and are rebased up.

**The refactor** (no behaviour change; function bodies moved verbatim):
- `setup()` in `canopen.cpp` is a fixed sequence of per-feature hooks: `add_*_od_()` (OD entries) -> `od.finalize()`
  -> `config_*_(cfg)` (`CO_config_t` counts / entries) -> `CO_new` / `CO_CANinit` -> `init_pdo_storage_()` ->
  `CO_CANopenInit` -> `init_*_()`. Optional features are `#ifdef USE_CANOPEN_*` around their hook calls. A PR adds
  its file plus one hook call per phase.
- `canopen.h` stays one class; its public and protected parts are grouped by feature under banners
  `// --- <Feature> (<file>) ---` (members of optional features inside their `#ifdef`).
- `receive_()` (driver) hands frames to CANopenNode, then to `dispatch_frame_()` (frames.cpp: listeners,
  heartbeat / EMCY callbacks); the loopback is the `next_local_` loop in `transmit()`.
- The OD handler API is split from the basic runtime OD: `od_handler.h` / `od_handler.cpp`; `ObjectDictionary` keeps
  `set_object()`, `write()`, `set_extension_()` and the trampolines in `// --- Handlers ---` blocks, plus 5 extension
  lines in `PendingEntry` / `finalize()` (PR 1 cut drops them: `extension` always 0, `ext` nullptr).
- `to_nmt_state()` is no longer `static` (used by `node_monitor.cpp`).
- Not split: `types.h` (one block per type, the PR 2 types are its last block), `__init__.py` (contiguous blocks;
  the shared `CONFIG_SCHEMA` / `to_code()` / `_final_validate` get single lines per PR), `od_test.cpp`,
  `loopback_test.cpp` (tests of several PRs, cut by `TEST`).
- Checked: `script/cpp_unit_test.py canopen` (34 tests), `script/test_build_components.py -c canopen -t <target>`
  for esp32-idf, esp32-ard, esp8266-ard, rp2040-ard (no warnings), the mapper on ESP8266 / ESP32-C3 (upstream venv)
  and STM32L4 (fork venv). Cost: flash +0.4-0.6 KB (hooks no longer inlined into `setup()`), RAM +8 B. clang-tidy: see
  "Upstream conventions" (clean after the port header move, 2026-11-01).
- When adding code to the hub from now on: put it in its feature's file and banner block, keep `setup()` /
  `process_()` / `dump_config()` to one hook call per feature, so the cut stays mechanical.

**Files** (`esphome/components/canopen/`):

| File | PR | Content |
|---|---|---|
| `canopen.cpp` | 1 | setup sequence, 0x1001 / 0x1014 / 0x1017, `CO_new` / `CO_CANopenInit`, `loop()` / `process_node_()`, own NMT state, `dump_config()` |
| `driver_can.cpp` | 1 | CANopenNode CAN driver (`CO_CAN*`), `on_frame_()`, `receive_()`, `transmit()` (+ loopback: 2) |
| `device_info.cpp` | 1 | 0x1000, 0x1008-0x100A, 0x1018, SDO server 0x1200 |
| `od.h` / `od.cpp` | 1 | runtime OD (`add_var/array/record/string`, `finalize()`, `find()`); `// --- Handlers ---` blocks: 4 |
| `types.h` | 1 | `NMTState`; `NodeEvent`, `FrameCallback`, `Emergency`, decoders: 2; `NMTCommand`, SDO types: 3; `PdoMapping`: 5 |
| `__init__.py` | 1 | schema / codegen; blocks of PRs 2-6 (see below) |
| `node_monitor.cpp` | 2 | 0x1016, `monitor_node()`, HB consumer callbacks, node events, `get_node_state()` / `is_node_online()` |
| `frames.cpp` | 2 | frame listeners, `send_frame()`, `dispatch_frame_()` (heartbeat / EMCY callbacks) |
| `sdo_client.cpp` | 3 | SDO client, 0x1280 |
| `nmt_master.cpp` | 3 | `send_nmt()` |
| `automation.h` | 3 | `SDOWriteAction` |
| `od_handler.h` / `od_handler.cpp` | 4 | `OdResult`, `OdStream`, `OdObject`; `set_extension_()`, `write()`, `read()`, `value_size()` |
| `canopen_node.h` / `canopen_node.cpp` | 4 | `CANopenNode` base class (profile API) |
| `pdo.cpp` | 5 | `configure_tpdo/rpdo()`, 0x1400-0x1A00 records, `CO_CANopenInitPDO`, processing, `request_tpdo()` |
| `pdo_storage.cpp` | 5 | 0x1010 / 0x1011, load / store / restore in preferences |

`__init__.py` beyond PR 1: PR 2 `HB_CLIENT_SCHEMA`, `validate_heartbeat_clients`, `heartbeat_clients` /
`on_node_event` / `on_heartbeat` / `on_emcy` (schema, `CALLBACK_AUTOMATIONS`, `to_code()`), `KEY_LOCAL_NODES` and the
local ring in `to_code()`, the duplicate node id check in `_final_validate`; PR 3 `_enable_once`,
`enable_sdo_client()`, `enable_nmt_master()`, `NMT_COMMANDS`, `response_timeout`, the `canopen.sdo_write` /
`canopen.nmt` actions; PR 4 `CANOPEN_NODE_SCHEMA`, `final_validate_canopen_node`, `register_node()`, the
`CANopenNode` class; PR 5 `enable_pdo()`; PR 6 `enable_sdo_block()`, `SDO_BLOCK_BUFFER_SIZE`, `sdo_block_size`
(schema, `to_code()`, the firmware-wide check in `_final_validate`). `_enable_once` comes with the first `enable_*()`
helper in the chain (PR 3).

**Line counts** (`scripts/pr_split_lines.py`, see below; non-blank lines, comments included, all `.cpp`, `.h`
and `__init__.py`; 2694 in all; the port header is in the CANopenNode fork since 2026-11-01):

| PR | Content | .cpp | .h | .py | Total | Comment-only | Tests (non-blank) |
|---|---|---|---|---|---|---|---|
| 1 | Node core: library, driver, basic OD, NMT + HB producer, SDO server, identity / device info | 393 | 182 | 133 | **708** | 76 | 259: `common.yaml` + `test.*.yaml` 130, `fake_canbus.h` 45, `od_test` 62, `__init__.py` 22 |
| 2 | Node monitoring, raw frames, heartbeat/EMCY triggers, loopback | 162 | 100 | 70 | **332** | 37 | 141: `frames_test` 59, `loopback_test` 82 |
| 3 | SDO client, NMT master, `canopen.sdo_write` / `canopen.nmt` | 279 | 163 | 115 | **557** | 36 | 102: `loopback_test` (SDO tests) 77, `sdo_reply_test` 25 |
| 4 | OD handler API (`OdObject`, `OdStream`, `write_od()`, `read_od()`, `value_size()`) + profile API (`CANopenNode`, `register_node()`) | 101 | 194 | 37 | **332** | 56 | 299: `canopen_node_test` 108, `od_test` (handler and OD access tests) 191 |
| 5 | Own PDOs + store/restore 0x1010/0x1011 | 243 | 77 | 10 | **330** | 37 | 202: `pdo_test` 103, `storage_test` 99 |
| 6 | SDO server block transfer (`sdo_block_size`) | 3 | 0 | 44 | **47** | 6 | 230: `sdo_block_test` |
| 7 | OTA on CiA 302-3 objects (`ota: - platform: canopen`), see below | 199 | 110 | 79 | **388** | 32 | 277: `ota/program_download_test` 245, `test-ota.*.yaml` 27, `ota/__init__.py` 5 |

Dependencies (why this order): 2 uses 1's NMT strings and frame path; 3's actions and local SDO transfers use the OD
(1) and the loopback (2, `loopback_test` SDO tests); 4's `CANopenNode` forwards the SDO client / NMT master (3); 5
maps OD entries with handlers (4, `request_tpdo(index, sub)` uses the extension); 6's test downloads into an
`OdObject` (4). `cia402` (old PRs 5 / 6, on top of #21533) is a separate chain on PR 4.

Notes:
- PR 1 is under the ~1000-line limit (~630 without comment-only lines). If it's still too much: identity / device
  info (`device_info.cpp` without 0x1200, ~70) can move to PR 2.
- PR 6 is small: fold it into PR 5, or keep it separate, since its download size cap needs CANopenNode's
  `CO_CONFIG_SDO_SRV_BLKSIZE` patch (fork branch `sdo-srv-blksize`), which upstream CANopenNode doesn't have yet.

**Line count script:** `scripts/pr_split_lines.py [<esphome>/esphome/components/canopen]` (default
`~/repos/esphome/...`) prints the table above and the tests per PR. It classifies lines by markers, not line numbers
(whole files, the banners, `#ifdef USE_CANOPEN_*` blocks, keyword lines, `types.h` declarations, `__init__.py`
block names and keywords; rules at the top of the script), so it survives edits; `--annotate <file>` prints a file
with the PR of every line. Add new files / blocks to its tables. Accuracy: a few lines.

**Cutting the branches** (next steps, in order):
1. Done: the refactor is committed on `canopen-full` (`e9f738ebf7`); keep it checked out in `~/repos/esphome` for the
   mapper builds here. Pushed to mrk-its/esphome (`origin/canopen-full`).
2. Branches `canopen-pr1-node` .. `canopen-pr6-sdo-block` from `upstream/dev`, stacked, one commit each. For PR N:
   take `esphome/components/canopen/` and `tests/components/canopen/` from `canopen-full`, delete the files of PRs
   > N, and remove the lines of PRs > N from the shared files (`--annotate` shows them: hook calls and
   `#ifdef` blocks in `canopen.cpp`, banner blocks in `canopen.h` / `od.h`, the `types.h` blocks, the `__init__.py`
   blocks and lines, `dispatch_frame_()` / the `next_local_` loop in `driver_can.cpp`, the extension lines in
   `od.*`); trim the tests the same way (test `__init__.py` enables only the `enable_*()` of PRs <= N; `common.yaml`
   only the options of PRs <= N; `TEST`s by the table above). The clang-tidy setup (`canopen-full` `62bd875`) is cut
   the same way: PR 1 adds the CANopenNode line to `platformio.ini` `lib_deps_base` (upstream CI runs clang-tidy on
   the files a PR changes; without it `CANopen.h` isn't found), and each PR adds the `esphome/core/defines.h` lines of
   the `enable_*()` it introduces:

   | PR | `defines.h` |
   |---|---|
   | 1 | none (the node core uses the `CO_CONFIG_*` defaults of `CO_driver_target.h`) |
   | 3 | `USE_CANOPEN_SDO_CLIENT`, `CO_CONFIG_SDO_CLI`, `CO_CONFIG_FIFO`; `USE_CANOPEN_NMT_MASTER`, `CO_CONFIG_NMT` |
   | 5 | `USE_CANOPEN_PDO`, `CO_CONFIG_PDO`, `CO_CONFIG_STORAGE` |
   | 6 | `CANOPEN_SDO_BLOCK_SIZE`, `CO_CONFIG_SDO_SRV`, `CO_CONFIG_SDO_SRV_BUFFER_SIZE`, `CO_CONFIG_SDO_SRV_BLKSIZE`, `CO_CONFIG_CRC16` |

   The last branch must equal `canopen-full` (`git diff canopen-pr7-ota canopen-full --
   esphome/components/canopen tests/components/canopen platformio.ini esphome/core/defines.h` shows only the Zephyr
   parts PR 7 leaves out, see [canopen_ota.md](canopen_ota.md), "Upstream").

   **PR 1 done (2026-11-01), pushed:** `canopen-pr1-node` in `~/repos/esphome`, `50448cf939` on `upstream/dev`
   `0548ad7a72` (first cut `144fce267f` on `27aa91bb9d`, rebased) (component, `tests/components/canopen`, `tests/component_tests/canopen`, CODEOWNERS, the
   `platformio.ini` line; 659 lines by the script), and in `~/repos/esphome-docs`, `bad076268` on `next` `231db3c44`
   (the page cut to PR 1, the two index links). Checked: `cpp_unit_test.py canopen` (2), the Python component tests
   (6), `test_build_components.py` for the four targets (only PR 1's four sources built, no warnings), clang-tidy
   esp8266-arduino-tidy and esp32-idf-tidy (4 files, clean; it only checks committed files, so run it after the
   commit), pre-commit. Cut by hand where the script's annotation is wrong (fix its rules before PR 2): the end of
   the raw frames block in `canopen.h` (`};`, `frame_listeners_`, heartbeat / EMCY callbacks: PR 2),
   `set_response_timeout()` / `response_timeout_ms_` (PR 3), the OD storage members in `od.h` (PR 1, only
   `extensions_` is PR 4), the `PendingEntry{..., 0}` extension initializers in `od.cpp` (PR 4), the loopback loop
   body in `transmit()` (PR 2). Also: the class comment and includes trimmed to PR 1, `OD_TPDO` / `OD_RPDO` left to
   PR 5, `_final_validate` left out (all its checks are PRs 2 / 6), `fake_canbus.h` moves to PR 2 (no PR 1 test uses
   it), `od_test.cpp` has `Strings` and `Record` only. Venv: `PYTHONPATH=<worktree>` so the scripts import the
   worktree's `esphome`, not the editable install of `~/repos/esphome`.
3. Each branch on its own: `script/cpp_unit_test.py canopen`, `script/test_build_components.py -c canopen -t
   <target>` for the four targets, clang-tidy on esp8266-arduino-tidy, esp32-idf-tidy,
   bk72xx-tidy and nrf52-tidy (with the branch's own part of the tidy setup, step 2), and the line count (`scripts/pr_split_lines.py` on the
   branch's tree: PR N's own lines are the "PR N" row).
4. Docs: re-split `~/repos/esphome-docs` (branch `canopen-full`, the whole page, matching this branch) the same way,
   one page section per PR.
5. Then: ~~delete the old `canopen-1-node` .. `canopen-4-profile` branches on mrk-its/esphome~~ (done 2026-11-01:
   `canopen-1-node` `acf8485713`, `canopen-2-frames` `09010a96e7`, `canopen-3-sdo` `933739c6eb`, `canopen-4-profile` `ab086aa045`), replace
   `docs/upstream_prs_1-3.md` with a description of the new branches, update CLAUDE.md, and before PR 1 goes out the
   open points of "Before PR 1" below (#21522, library source, CI boilerplate).

**PR 7: OTA in the hub** (implemented, on `canopen-full` since 2026-11-02; plan in [canopen_ota.md](canopen_ota.md),
"Upstream"): `ota: - platform: canopen` on the CiA 302-3 program download objects (`0x1F50` / `0x1F51` / `0x1F56` /
`0x1F57`) instead of the mapper's `0x3000`, which stays in the manufacturer area only for the old component. The mapper
then has no OTA. Branch `canopen-pr7-ota`, stacked on PR 6.

### SDO client only first (2026-11-05) — current plan
**Why.** A hub without `node_id` is a self-contained first PR: an SDO client that reads and writes objects of existing
CANopen devices (drives, sensors, I/O modules) from automations, without being a CANopen node. The node itself (NMT,
heartbeat, SDO server, identity, OD) follows in PR 3, after the raw frames (PR 2). Implemented on branch
`feature/canopen-sdo-client-only` in `~/repos/esphome` (`0c6e7cad40`), merged into `canopen-full` (`85ca51ceaf`), and
in `~/repos/esphome-docs` (same branch name, `bae1c36ff`). The old PR 1 (`canopen-pr1-node`, the node core, see "New
split") is superseded: PR 1 is cut anew, see "PR 1 as cut" below.

**What it is.**
- `node_id` is optional. Without it, `setup_client_only_()` runs `CO_new()` with only `CNT_SDO_CLI = 1`, then
  `CO_CANinit()` and `CO_SDOclient_init()` with node id `NO_NODE_ID` (0), on an OD that holds only `0x1280`. There is
  no `CO_CANopenInit()`: CiA 301 ids are 1..127, and LSS "unconfigured" stops it before the client. `process_()`
  runs `CO_CANmodule_process()` + the client.
- `USE_CANOPEN_NODE` (set if any hub has a node id) compiles the node path. Without it CANopenNode's node code isn't
  linked: ESP8266 (d1_mini + mcp2515, one `canopen.sdo_write`): node 292,095 B flash / 29,384 B RAM, client-only
  281,907 B / 29,372 B; STM32L4 (nucleo_l476rg, zephyr_can, fork venv): node 93,880 B flash, client-only 85,172 B
  (RAM 18,176 B both: Zephyr's static heap). CANopenNode symbols on ESP8266 drop from 12.2 KB (`CO_CANopenInit`, `CO_process`, NMT, HB consumer,
  EM, SDO server) to 6.0 KB (driver, SDO client, fifo, OD access).
- `sdo_client:` option: unset = auto (compiled in if an action or component uses it), `true` = always (lambdas),
  `false` = never (an error if something uses it). Firmware-wide, like `sdo_block_size`.
- `canopen.sdo_read` action (`on_value` with `x`, `on_error`).
- Allowed without `node_id`: `canbus_id`, `response_timeout`, `sdo_client`, `on_heartbeat` / `on_emcy` (raw frames),
  the SDO actions and lambdas. Node options are rejected on the raw config (before the schema's defaults);
  `require_node_id()` rejects profile components, OTA and `canopen_entities`; `canopen.nmt` fails in codegen.
- Node monitoring (`heartbeat_clients`, `on_node_event`) stays node-only: CANopenNode's HB consumer needs an
  initialised `CO_EM_t`, and `CO_EM_init()` rejects node id 0 (the workaround would be a fake id with the EMCY
  producer disabled). If needed later: our own monitor on top of the decoded heartbeat frames.

**New split** (2026-11-05; `scripts/pr_split_lines.py` on `feature/canopen-sdo-client-only` `0c6e7cad40`; non-blank
lines, comments included, all `.cpp`, `.h` and `__init__.py`; 2945 in all):

| PR | Content | .cpp | .h | .py | Total | Comment-only | Tests (non-blank, today's files) |
|---|---|---|---|---|---|---|---|
| 1 | SDO client (values, buffers, `cancel_sdo()`), `canopen.sdo_read` / `canopen.sdo_write`; a hub without `node_id` | 473 | 260 | 165 | **898** (+~30) | 87 | 254 (`common.yaml`, `fake_canbus.h`, `sdo_reply_test`, `__init__.py`) + a new client test |
| 2 | Raw frames (`add_frame_listener()`, `send_frame()`), `on_heartbeat` / `on_emcy`, `NMTState` | 61 | 77 | 17 | **155** | 17 | 59: `frames_test` |
| 3 | Node core: `node_id`, runtime OD, NMT + HB producer, SDO server, identity / device info, `sdo_client:`, `USE_CANOPEN_NODE`; several local nodes (loopback) | 257 | 144 | 184 | **585** | 65 | 329: `client_only_test` 108, `loopback_test` 159, `od_test` (strings, record) 62 |
| 4 | Node monitoring (`heartbeat_clients`, `on_node_event`), NMT master (`canopen.nmt`) | 124 | 44 | 86 | **254** | 23 | none yet (hardware-tested only) |
| 5 | OD handler API (`OdObject`, `write_od()` / `read_od()`), profile API (`CANopenNode`, `register_node()`, `require_node_id()`) | 99 | 154 | 47 | **300** | 45 | 299: `canopen_node_test` 108, `od_test` 191 |
| 6 | Own PDOs, store / restore 0x1010 / 0x1011 | 243 | 77 | 10 | **330** | 37 | 202: `pdo_test` 103, `storage_test` 99 |
| 7 | SDO server block transfer (`sdo_block_size`) | 3 | 0 | 30 | **33** | 3 | 230: `sdo_block_test` |
| 8 | OTA on CiA 302-3 objects (`ota: - platform: canopen`) | 199 | 110 | 81 | **390** | 33 | 278 |

Per PR:
- **1** is the whole SDO client, with no node: `setup()` is today's `setup_client_only_()`, no `node_id`, no
  `NO_NODE_ID` / `is_client_only()`. ~930 lines with the fixed entry (~840 without comment-only lines), under the
  ~1000-line limit. The buffer transfers and `cancel_sdo()` (~80 lines) stay in: they are interleaved with the value
  path in `start_sdo_()` / `process_sdo_()` / `finish_sdo_()`, so cutting them out would mean rewriting those
  functions, not deleting lines. `CO_SDOclient_init()` needs `0x1280` as an `OD_entry_t`: PR 1 has a fixed entry (an
  `OD_obj_record_t[4]` + `OD_entry_t` + `OD_t`, ~30 lines, not in the tree yet, counted as "+~30"); PR 3 replaces it
  with the runtime OD (`od.*`, 139 lines), which PR 1 would otherwise carry for one record. Files: `driver_can.cpp`
  (without the loopback and `dispatch_frame_()`), `sdo_client.cpp`, `canopen.h`, `canopen.cpp`, `types.h` (the SDO
  types), `automation.h`, `__init__.py` (library, `canbus_id`, `response_timeout`, `canopennode_path`,
  `enable_sdo_client()`, the two actions). Tests: the existing client tests run against a local node (PR 3), so PR 1
  needs its own (~100 lines): the hub against canned server replies through `TestHub::frame()` (expedited and
  segmented read / write, timeout + abort frame, a server abort, the queue, cancel, the actions' codegen).
- **2** works without `node_id`: frames reach `dispatch_frame_()` from the driver's `receive_()`; listeners and
  `send_frame()` need only the CAN module. It brings `NMTState` and `nmt_state_to_string()` (the heartbeat trigger's
  state); the node's own NMT state uses them from PR 3 on.
- **3** makes the hub a CiA 301 node. Also: local SDO transfers (requests to the own id served from the OD), the
  `sdo_client:` option (auto / `true` / `false`; it only means something once a hub can be without the client), the
  node-option checks for a hub without `node_id`, and the loopback between local nodes (24 lines: it only matters
  once hubs answer each other, and its duplicate check needs node ids).
- **4**: both need a node (CANopenNode's HB consumer needs the EM object; the NMT master the NMT object).
- **5**: `require_node_id()` comes with its first user (profile components); OTA and `canopen_entities` use it later.
- **7** needs CANopenNode's `CO_CONFIG_SDO_SRV_BLKSIZE` patch (fork branch `sdo-srv-blksize`).

Dependencies: 2 needs only 1 (the driver); 3 adds the node next to 1's client-only setup and uses 2's `NMTState`;
4 needs the node (3) and decodes heartbeats with 2's types; 5's OD API needs the runtime OD (3) and `CANopenNode`
forwards the SDO client / NMT master (1, 4); 6 maps OD entries with handlers (5); 7's test downloads into an
`OdObject` (5); 8 uses block transfer (7) and handlers (5).

`defines.h` lines (the clang-tidy setup, see "Cutting the branches"): PR 1 `USE_CANOPEN_SDO_CLIENT`,
`CO_CONFIG_SDO_CLI`, `CO_CONFIG_FIFO`; PR 3 `USE_CANOPEN_NODE`; PR 4 `USE_CANOPEN_NMT_MASTER`, `CO_CONFIG_NMT`; PR 6
`USE_CANOPEN_PDO`, `CO_CONFIG_PDO`, `CO_CONFIG_STORAGE`; PR 7 `CANOPEN_SDO_BLOCK_SIZE`, `CO_CONFIG_SDO_SRV*`,
`CO_CONFIG_CRC16`. The `platformio.ini` CANopenNode line goes with PR 1.

Accuracy: `scripts/pr_split_lines.py` classifies by markers (rules at its top); a line belongs to the newest PR among
its function, `#ifdef`s, banner and keyword. The `setup()` / `process_()` lines that pick the client-only path count
as PR 1 although they only exist from PR 3: a few lines each way. The cut follows the old "Cutting the branches"
steps (PR N: delete the files and lines of PRs > N, `--annotate` shows them).

#### PR 1 as cut (2026-11-07)
- **Branches** (pushed to mrk-its, one commit each, no Claude attribution in commits or PR text):
  `canopen-pr1-sdo-client` in `~/repos/esphome`, `79f5437814` on `upstream/dev` `a9ba8e9743`, "[canopen] Add CANopen
  SDO client component (on CANopenNode)"; `canopen-pr1-sdo-client` in `~/repos/esphome-docs`, `0bbf9e1bf` on
  `origin/next` (the page cut to PR 1, the index and CAN Bus "See Also" links; `node script/lint.mjs` clean).
  PRs (opened 2026-11-07 as drafts, ready for review since 2026-11-09): esphome/esphome#22410 and esphome/esphome.io#8059, both `[canopen] Add CANopen SDO
  client component`, linked to each other. The esphome PR says it replaces #21522 (kj4dxk's hub); Test Environment:
  the STM32 nucleo, in a sentence (no box ticked).
  Until reviews start, fixes are amended and force-pushed; afterwards only new commits.
- **Size:** the esphome bot labels a PR `too-big` above 1000 changed lines outside `tests/` (additions minus
  deletions, blank lines and comments included; `TOO_BIG_THRESHOLD` in `.github/workflows/auto-label-pr.yml`). The
  first cut had 1031; trimmed (2026-11-07) to 901 by leaving out `canopennode_path:`, the SDO client switch
  (`enable_sdo_client()`, the `CANopenData` dataclass, the `USE_CANOPEN_SDO_CLIENT` `#ifdef`s: the client is all PR 1
  runs) and `cancel_sdo()` (with the abort guard, `abort_sdo_()`, `discard_pending_tx()` and `sdo_cancel_test.cpp`).
  Count with `git diff --numstat upstream/dev -- . ':!tests'`. Keep fixes during review under the limit. 940 after the
  first review's fixes (below).
- **First review** (`esphbot`, 2026-11-08; fixes on `canopen-full` `376dcbdaa6`, `75cf87553a`, in the cut as one new
  commit): with TX refused, CANopenNode returns `CO_SDO_RT_transmittBufferFull` and never reports the timeout, so a
  request and the queue behind it waited until the bus came back (the bot missed this one and misread the late write).
  A frame unsent at the response timeout now ends the request with `TRANSPORT_ERROR`; a failed request drops its unsent
  frame, also an abort that the controller refused at that moment. `CO_CANtxBufferInit` keeps `CANtxCount` right
  (`CO_SDOclient_setup()` re-inits the buffer for every request: no `CO_CONFIG_FLAG_OD_DYNAMIC`). So
  `discard_pending_tx()` is in the cut after all. Also: `write_sdo()` rejects values that don't fit, rejections are
  logged, frames over 8 bytes are ignored, an idle client-only hub returns early from `process_()`.
  `read_sdo_data()` takes the caller's buffer and capacity (no hub buffer, no option); `SDORequest::data` is non-const
  because of that (clang-tidy flags a `const` field written through a cast), `write_sdo_data()` has the one cast. Not
  done: `cancel_sdo()`'s abort on `canopen-full` can still block the queue while TX is stuck (`sdo_abort_pending_`); give
  it the same limit when PR 2 is cut.
- **Content:** `__init__.py` (library + the two `CO_CONFIG_*` build flags in `add_canopennode_library()`,
  `canbus_id`, `response_timeout`, the two actions), `automation.h`, `canopen.cpp/.h`, `driver_can.cpp`,
  `sdo_client.cpp`, `types.h` (SDO types only); CODEOWNERS, `platformio.ini` CANopenNode line, `defines.h`
  (`CO_CONFIG_SDO_CLI 0x03`, `CO_CONFIG_FIFO`). Tests: `sdo_client_test.cpp` (new, 10 tests against canned server
  replies through `TestHub::frame()`), `sdo_reply_test.cpp`, `test_canopen.py` (codegen, defaults,
  `response_timeout` range, `sdo_write` values, `node_id` rejected); 14 C++ + 14 Python tests, the four build targets
  and clang-tidy (4 envs) clean. Upstream CI also runs clang-tidy on the test files, with esp32-idf-tidy only (the
  other tidy envs have no gtest): `script/clang-tidy -e esp32-idf-tidy esphome/components/canopen
  tests/components/canopen`, after committing. The first push failed it on `sdo_client_test.cpp`: fixture members
  need the trailing `_` too (`bus_`, `hub_`, `requests_()`; gtest's `SetUp()` / `TEST_F` are accepted).
- **Differences from `canopen-full`, kept in the cut** (later cuts and the rebase after the merge must know them):
  - fixed `0x1280` entry (`sdo_cli_record_`, `sdo_cli_entry_` in `canopen.h`; no `OD_t`: `CO_SDOclient_init()` gets
    `nullptr`, since `8256c1ed26`) instead of the runtime OD; PR 3 replaces it;
  - CANopenNode from the registries (since `7e4ac679d0`): `esphome/CANopenNode@4.1.0-esphome.2`, on ESP32 as the IDF
    component `esphome/canopennode`; `canopen-full` still pins the git commit (`CANOPENNODE_SOURCE`, `platformio.ini`
    `lib_deps_base`). Until ported, a CANopenNode bump means a new registry release for the cut;
  - the actions are `automation.ApplyAction`s (`apply_statements_action`) calling `write_sdo()` / `read_sdo()` with a
    reply function from the header templates `sdo_write_reply<ON_SUCCESS, ON_ERROR>` / `sdo_read_reply<ON_VALUE,
    ON_ERROR>` (automations built with `build_trigger_callback`, `nullptr` when not configured); `canopen-full` still
    has the `SDOWriteAction` / `SDOReadAction` classes (since `5cd20a9dc8`..`5f2c254e14`);
  - `CO_CONFIG_SDO_CLI=0x03` (no local transfers); PR 3 brings back `0x0B`;
  - no `canopennode_path:`, no SDO client switch (`enable_sdo_client()`, `CANopenData`, `USE_CANOPEN_SDO_CLIENT`),
    no `cancel_sdo()`: see "Size". `cancel_sdo()` and `sdo_cancel_test.cpp` go to PR 2, the client switch (with
    `sdo_client:`) to PR 3, `canopennode_path:` to whichever PR needs it next;
  - the canbus is a constructor parameter since bdraco's commits (below); `canopen-full` takes `(canbus, node_id)`.
- **esphome conventions** (its AGENTS.md, checked before opening): fixed on `canopen-full` (`bf0f7c20cf`) and folded
  into the cut: no heap after `setup()` for the SDO client's reads (a member buffer then, the caller's buffer since the
  first review), the abort guard counts down
  with the client's `dt` instead of `millis()` (the OTA phases use `App.get_loop_component_start_time()`), codegen
  state in a `CANopenData` dataclass under `DOMAIN`, type hints everywhere; `script/build_language_schema.py`
  extracts the whole schema. OTA clang-tidy findings fixed on `canopen-full` (`4676376312`).
- **Hardware test** (nucleo, 2026-11-07; config `test/stm32l4_sdo_client.yaml` + `test/sdo_client_test.h`, not
  committed: it built a temporary copy of the cut with `set_canbus()` back on `add_callback()`): over 20 house nodes read (device name, software version, identity,
  heartbeat time); `canopen.sdo_read` value, size mismatch, missing object (`0x06020000`), absent node (timeout
  `0x05040000`), queue full (16 of 17); `canopen.sdo_write` (B1 toggles dimmer4 light5: state `0x2051:1`, command
  `0x2052:1`). The only aborts (`0x05040001` / `0x06040043`, ~7% of requests on 0x61 and 0x67, none with can2mqtt
  stopped) came from can2mqtt polling the same nodes' default SDO channel for remote logs (the stm32 nodes flashed
  since 2026-11-03 have `remote_logs` from `common/stm32_zephyr_can_node.yaml`, level INFO, and can2mqtt's `AUTO`
  subscribes to them): two SDO clients on one server channel collide. Worth a sentence in the docs (another SDO
  master on the bus: handle `on_error`).
- **Hardware test of the first review's fixes** (nucleo, 2026-11-08; the cut with the same compat patch, a test-only
  `CANtxCount` getter and a temporary test header, in the session's scratchpad). `can_stop()` from a lambda makes the
  real driver refuse frames (no wiring touched); only absent node 0x7E is written to, dimmer4 only read. To see whether
  a discarded frame went out (the host has no CAN interface), the nucleo reads the unusual `0x1018:4` and listens on
  `0x5B7`: 1 answer in the control read, 0 after the bus came back.
  - `tx_buffer_size: 0`: three stuck requests ended with `TRANSPORT_ERROR` at +516, +1074, +1631 ms, 24 canbus warnings
    each (72); `CANtxCount` 0; a read after `can_start()` succeeded in ~17 ms. An abort refused at the timeout: one
    warning, then dropped.
  - `tx_buffer_size: 32`: `zephyr_can` takes the frame and drops it itself while stopped (`send_next_()`), so the hub
    never sees a refusal: the same requests end with an ordinary `TIMEOUT`, no warnings. `TRANSPORT_ERROR` only appears
    without the queue or with the queue full. A disconnected bus (frames stuck in the mailboxes, not a stopped
    controller) wasn't tried.
  - Found here, not by the gtests: a read whose last frame overflows the buffer gave `SDO_RESULT_ABORT` with code 0,
    because `CO_SDOclientUpload()` clears it when the transfer is already over. Fixed (`ff116ed538`, in the cut).
  - One pass over the house nodes: all reads succeeded, no aborts or timeouts.
- **Copilot review** (2026-11-19, review 5966531279, 17 comments; fixes on `canopen-full` `eddd646f1f`, in the cut as
  `2318307be0`, docs `a73f911ed` / `ef24ebff1`). Before it, someone merged `dev` into the cut from the GitHub UI
  (`a955eae7c0`): `git pull --ff-only` before working on the cut. Fixed:
  - one hub per canbus (the cut); on `canopen-full` one hub *without `node_id`* per canbus (node hubs plus one
    client-only hub on a bus stay valid, as the loopback test uses). Two clients on one bus would share COB-IDs;
  - `process_()` runs the SDO client before the TX flush (`CO_CANmodule_process()` / `CO_process()`). Otherwise a frame
    still waiting at the deadline went out in the flush and the caller then got `TIMEOUT`. The abort guard
    (`canopen-full`) now counts from the flush that sent the abort (a pending abort found sent left `dt` ago);
  - stale replies: while a request runs, initiate responses / aborts from its server that name another object are kept
    from CANopenNode's client (`sdo_reply_is_stale_()`, checked in `receive_()` for the client's rx buffer only, so
    frame listeners and other local hubs still see them). CANopenNode takes any abort without checking the object;
  - a value read drains the fifo after every segment (before: only when full or at the end) and stops past 4 bytes:
    `SIZE_MISMATCH`, server aborted with `0x06070012`; `SDO_MAX_VALUE_UPLOAD` removed;
  - `write_sdo()` masks a sign-extended value to `size`, so the reply's value is zero-extended;
  - requests before `setup()` (default `on_boot`, priority 600, runs before the hub's 400) are queued instead of
    rejected; `fail_setup_()` marks the hub failed with a reason and rejects the queue. Not done by raising the hub's
    priority: `canopen_entities` (`PROCESSOR + 1`) adds OD entries before the hub's setup;
  - CAN FD frames ignored (`frame.fd`). For that, `canopen-full` moved to `add_frame_callback()` (the fork venv's
    esphomeplatformzephyr has it now), so that difference from the cut is gone;
  - `on_success` / `on_value` / `on_error` take one automation (`validate_automation(single=True)`, one `Trigger`
    each, as `http_request`); shared `SDO_ACTION_SCHEMA` / `_sdo_action_to_code()`; `final` actions;
    `PROGMEM_STRING_TABLE` result names; reasons for `mark_failed()`; one `ESP_LOGCONFIG`; init log verbose;
  - tests: `SdoTest` fixture in `fake_canbus.h`; `sdo_client_test.cpp` now also on `canopen-full`; new tests for the
    deadline flush, stale replies, long value reads, signed raw writes, requests before setup and FD frames
    (`USE_CANBUS_FD` defined in the unit test build); `canopen_node_test.cpp` gets its synchronous rejections from a
    failed hub now.
  Documented, not fixed: a frame the controller has taken can't be recalled (MCP2515 sets TXREQ and returns OK), so a
  write that failed with `TIMEOUT` / `TRANSPORT_ERROR` may still have been carried out (`types.h`, docs); also
  "another SDO client on the same node: handle `on_error`" (the can2mqtt collision above). Not changed (answered):
  lambda results narrowed to the field type (every `TEMPLATABLE_VALUE` does that); ESP32 `tx_enqueue_timeout`
  blocking (only with a full TWAI queue, i.e. a dead bus; the platform's setting).
  Size: 985 lines (`git diff --numstat $(git merge-base upstream/dev HEAD) -- . ':!tests'`; 941 before). Fetch
  `upstream dev` first: against a stale `upstream/dev` the count includes the merged `dev`. Tests: `canopen-full`
  95 C++ + 43 Python, the cut 31 C++ + 18 Python; the four build targets on both. clang-tidy clean (esp32-idf-tidy with the tests, esp8266-arduino-tidy on the component; both branches). Build checks here: `stm32l4_loopback.yaml` (fork venv), `esp8266_mapper.yaml` (upstream venv, updated to 2026.12.0-dev: the old one had no `add_frame_callback()`).
  Hardware test (nucleo, 2026-11-19; `test/stm32l4_sdo_client.yaml` + `test/sdo_client_test.h`, not committed, hub from
  a worktree of the cut, no compat patch any more, `tx_buffer_size: 0`): the `on_boot` read at the default priority
  succeeded (it was REJECTED before); the fixed cases as before (value, size mismatch, `0x06020000`, absent node
  timeout, queue 16 of 17); a value read of `0x1008` gives SIZE_MISMATCH; the pass over the house nodes 124 successes,
  no aborts or timeouts. Stall: the node pass paused and the queue drained first (in the first run a node-pass request
  was ahead of the stalled one, so its deadline came after `can_start()`), `can_stop()`, a read of `0x1018:4` from
  dimmer4, `can_start()` from that read's callback, i.e. in the loop of its deadline, before the hub's TX flush:
  TRANSPORT_ERROR, 0 answers on `0x5B7` afterwards, a control read then got 1.
- **Second Copilot review** (2026-11-20, review 5984666368: 4 new comments, 7 "previously missed" in unchanged code;
  Copilot resolved 16 of the first round's 17 threads itself, also the two answered without a change; `on_boot` stayed
  open). Fixes on `canopen-full` `4c59c79c01`, in the cut as `1f3736764b`:
  - hub setup priority `HARDWARE - 1` (after the CAN bus, before default `on_boot`); the queue stays. On
    `canopen-full` the setup-order contract in `canopen.h` now says "above `HARDWARE - 1`"; `canopen_entities` (adds
    its OD entries in `setup()`) moved from `PROCESSOR + 1` to `HARDWARE - 0.5`. Everything else here registers from
    codegen (OTA, remote logs, `register_node()`);
  - a reply while the client's own frame is unsent is dropped (`sdo_reply_is_stale_()`): it can't be for that request.
    A late ack for the same object after the request went out can't be told apart (no transaction id in SDO);
  - `CO_CONFIG_SDO_CLI_BUFFER_SIZE=7`: the fifo holds one segment, so CANopenNode returns `uploadDataBufferFull` instead
    of requesting the next segment; `process_sdo_()` drains, checks the capacity, then calls `CO_SDOclientUpload()`
    again in the same pass (`canopen-full` also for `waitingLocalTransfer`). An oversized read sends initiate, one
    segment request, abort. The flag reaches CANopenNode's sources on zephyr too (`compile_commands.json` of
    `stm32l4_loopback.yaml`), it changes `CO_SDOclient_t`;
  - `process_(bool from_frame)`: on the frame path the client's time doesn't run unless its frame is unsent or no request
    runs (a received frame may come before a reply queued behind it after a long loop). The cut's `last_us_` is only the
    SDO clock; `canopen-full` has `sdo_last_us_` (CO_process() keeps `last_us_`). Starting a request from the frame path
    applies real time too, otherwise the next `loop()` would see the whole idle time;
  - the response timer restarts when the flush sends a queued frame (`canopen-full` flushes itself before `CO_process()`,
    which flushes again);
  - a server abort with `0x05040000` is ABORT; `<cstring>`; `uint8_t` queue counters; `put_le32()`;
  - not changed (answered): canbus' send-failure warnings while the controller refuses frames.
  Clang-format: the hook pins v23.1.3 (= `/usr/bin/clang-format`); the venv's `clang-format` is 13 and wraps
  differently, don't use it. Size 993 after tightening comments (1009 before; the same comment edits on `canopen-full`
  where the text matched). Tests: `canopen-full` 100 C++ + 43 Python, the cut 36 + 18, mutation-checked; builds and
  clang-tidy clean on both; here `stm32l4_loopback.yaml`, `stm32l4_canopen.yaml` (canopen_entities, new priority) and
  `esp8266_mapper.yaml` build. Hardware (nucleo, same test): all cases as before, incl. the stall and the `on_boot`
  read; the house pass 124 successes, segmented strings (up to 29 bytes) intact with the 7-byte fifo.
- **bdraco's commits** (2026-11-20, pushed to the cut on top of `1f3736764b`, last `1036a42cff`; the `dev` merges
  not counted). Ported to `canopen-full` by hand (the cut's code doesn't fit the node core 1:1):
  - `aba4d27b73`: the queue is a `StaticRingBuffer`, the request's `value` / `data` share a union, members reordered;
    an idle hub turns its loop off (`enable_loop()` in `queue_sdo_()` and `CO_CANsend()`); `SDOActionBase` for both
    actions; `CONF_CANBUS_ID` from canbus; `sdo_write` values not masked in codegen (`write_sdo()` masks); tests:
    one table for the rejected requests, `SdoTest` for all, YAMLs on `packages:`;
  - `580d5a6ced`: the canbus is a constructor parameter, the frame callback registered in `setup()`; `drain_upload_()`,
    one `sdo_len_`, `encode_value_le`; `test.esp32-ard.yaml` dropped;
  - `ba59c24159`: no `USE_CANBUS_FD` in the shared unit test binary (it broke the canbus tests);
  - `4c2cce3918`: the SDO client's time comes from the loop clock (`App.get_loop_component_start_time()`, ms) and runs
    only in `loop()`; a reply received in `on_frame_()` is processed at once and restarts the clock (CANopenNode
    restarts its response timer for it). Queued frames are flushed only from `loop()` (on ESP32 a refused frame can
    block for the controller's enqueue timeout). The tests move the loop clock (`advance_()`) instead of sleeping;
  - `ad1a963910` + `1036a42cff`: `Canbus::rx_capped()` (the last receive pass stopped at `max_frames_per_loop`);
    while a sent request waits for its reply, the hub holds its time until a pass drains, at most one response
    timeout past the deadline. Not for an unsent frame (no reply can wait for it);
  - `26554f9074`: superseded by the loop clock.
  Size of the cut after them: 991 lines outside `tests/` (993 added, 2 removed in canbus).
  Their tests fail clang-tidy (`readability-static-accessed-through-instance`: `this->advance_()` on a static): the
  PR's CI is red on "clang-tidy for ESP32 IDF 3/5 and 5/5" (run 44655413694). Fixed on `canopen-full` (`846650b945`,
  `advance_()` without `this->`) and on the cut (`99b2327702`, CI green).
  - **On `canopen-full`, differently:** a node still runs `CO_process()` for every received frame: CANopenNode keeps
    one received frame per RPDO (`CO_PDO_receive()`), so processing only in `loop()` would apply only the last of
    several frames of one COB-ID in a canbus pass (events, MPDO). The flush follows bdraco: `CO_CANmodule_process()`
    does nothing while `in_frame_` (CO_process() calls it), so queued frames are retried only from `loop()`; frames
    made while handling a received one go out at once only while nothing is queued. A node's loop never idles
    (heartbeat, PDO timers): `idle_()` is for client-only hubs. Two clocks: `CO_process()` / PDOs keep `micros()`
    (`last_us_`, inhibit times in 100 us), the SDO client `sdo_last_ms_`. `receive_()` stays (loopback, frame
    listeners); a reply from another local node gets no clock restart. `cancel_sdo()` turns the ring once (no erase)
    and enables the loop for the abort guard. `rx_capped()` through a C++20 `requires` helper in `canopen.cpp`, false
    on a canbus without it: the hub copy in esphome-canopen still builds on ESPHome installs without PR 1 (upstream
    venv, esphomeplatformzephyr). Remove the helper once `rx_capped()` (split out as esphome/esphome#23247, merged
    into `dev` 2026-11-24) is in a release. The house nodes load `canbus` from the
    `~/repos/esphome` checkout (`common_canopen_entities.yaml`), so they have the real `rx_capped()`.
  - New tests on `canopen-full`: `processing_test.cpp` (a burst of two RPDOs on one COB-ID is applied twice; a node
    doesn't retry queued frames on a received frame; the client-only loop idles; a node's doesn't), mutation-checked;
    `advance_loop_clock()` in `fake_canbus.h` for the other fixtures.
  - Hardware (nucleo as node 0x7f, `test/stm32l4_biuro_switch.yaml`, hub and canbus from the `canopen-full`
    checkout, `sdo_block_size: 60`, 2026-11-20): flashed over USB, operational, can2mqtt reads its version. CAN OTA
    from can2mqtt: 125,603 B in ~47 s (2.67 KB/s; 2.62 KB/s on 2026-11-02 before the change), no gap over 1 s between
    the kB log lines, MD5 OK, new image booted and confirmed by `safe_mode`. (A "B1 pressed" line at the start of a
    serial capture is old output the ST-Link held until the port was opened: the light didn't change then.)
  - Hardware, PR 1 only (`test/stm32l4_biuro_switch_sdo_client.yaml`: the cut's hub without `node_id`, canbus from
    `canopen-full`, B1 reads dimmer4 light5's state 0x2051:1 and writes the opposite to 0x2052:1): installed by CAN OTA
    through the node image above (can2mqtt finds the image by a hand-made `node_id: 7f, ver: ..` string), 94,019 B in
    ~35 s, booted as a client-only hub, confirmed by `safe_mode`. Two B1 presses: light off, then on (HA history at
    the same times), no SDO errors. That image has no CAN OTA: back to the node firmware over USB.
  - The last Copilot review (on `1036a42cff`) listed two "open issues": the two threads of its 2026-11-19 review,
    fixed in `1f3736764b` and answered but never resolved; resolved 2026-11-20, with a note on the timer one (the
    reset is in `loop()`, after the flush, not in `CO_CANsend()`). Its summary ("late SDO segments can abort
    unrelated queued requests") pointed at a gap the threads didn't cover: a segment response (scs 0/1) of a
    timed-out transfer, arriving after the next request to that node went out, made CANopenNode abort that request
    (`CO_SDO_AB_CMD`). `sdo_reply_is_stale_()` drops it while the request waits for its initiate response
    (`canopen-full` `261dee2ea8`, cut `51fdef96f1`; test `SegmentResponseWhileWaitingForTheInitiateOneIsIgnored`,
    mutation-checked). Cut size: 994 lines outside `tests/` (996 added, 2 removed in canbus).
  - Copilot review of 2026-11-21 (on `e725be04af`, the user's note in `types.h` on late responses for the same object),
    8 findings. Fixed (`canopen-full` `e83eb75e7e`, cut `3c5694c5b3`, docs `75b0e628a` on the PR branch and on docs
    `canopen-full`): a failed CAN bus fails the hub in `setup()` (requests made before it are rejected, not timed out;
    test `FailedBusFailsTheHub`, mutation-checked); `read_sdo<T>()` static_asserts a 1/2/4-byte `T` (`sizeof(T)` was
    narrowed to `uint8_t`); the dead `is_failed()` guard in `on_frame_()` removed; the callback contract names
    `setup()` as the other source of REJECTED; type annotations in the test codegen override. Declined with replies:
    canbus's warning per refused retry (canbus's, bounded by the response timeout), runtime tests of the action
    triggers (codegen test + SDO client gtests), stale segment replies for another object (dropped while waiting for
    the initiate response since `51fdef96f1`; a server answers in order). Cut size: 999 lines outside `tests/` (1001
    added, 2 removed in canbus).
  - **To discuss with bdraco when PR 3 is cut** (not raised on PR 1: the cut never flushes on the frame path): the
    node's per-frame `CO_process()` without the flush, above. Draft comment:
    > For a node hub I'd like to keep running `CO_process()` for every received frame, not move it into `loop()`.
    > CANopenNode keeps a single receive buffer per RPDO (`CO_PDO_receive()` copies into `CANrxData[0]` and sets
    > `CANrxNew`): if one COB-ID arrives twice in the same canbus pass, only the last copy would be processed. That's
    > harmless for state PDOs but loses data for event-like ones (commands, press/release, MPDOs where several objects
    > share one COB-ID). `CO_process()` flushes the TX queue itself, so our `CO_CANmodule_process()` does nothing while
    > a received frame is processed: queued frames are retried only from `loop()`, as in PR 1, and new frames go out
    > immediately only while nothing is queued. A stuck TX costs at most one refused send per pass. Does that match
    > what you had in mind, or would you rather have node hubs process only in `loop()`?

- **bdraco's second round** (2026-11-23/24, J. Nick Koston, 14 commits on the cut `7e4ac679d0`..`4acf244406`, after
  the user's dev merge `974feb0b6e` the head; **not ported to `canopen-full` yet**). Driven by a Copilot review of
  2026-11-24 (two findings, both fixed by him) and esphbot reviews:
  - `7e4ac679d0`: CANopenNode `4.1.0-esphome.2` from the registries (PlatformIO `esphome/CANopenNode`, IDF component
    `esphome/canopennode` on ESP32), the git pin dropped from `platformio.ini` (`[common:idf-component-libs]` now);
  - `8dba7ba846`, `6baffdfcaf`: `SDORequest` packed (`len` is `uint16_t`, next to `index`), driver checks that
    `CO_new()` makes unneeded dropped, one-line tests trimmed;
  - `06714d7b02` (Copilot): `write_sdo_data()` rejects more than 65535 bytes, `read_sdo_data()` clamps the capacity to
    65535 (`size_t` was narrowed to `uint16_t`: 65,537 bytes went out as 1, a 65,536-byte buffer became 0);
  - `e24ea6ce21` (Copilot): `sdo_reply_is_stale_()` drops an initiate response for the other direction (a late write
    ack while a read of the same object waits, and the reverse). Late aborts still match by object only;
  - `5cd20a9dc8`, `60aea80f2d`, `0a428b7866`, `8dc2548009`, `5f2c254e14`: the actions as `apply_statements_action`
    with the reply templates in `automation.h` (see the differences above); YAML schema and the automation variables
    unchanged; `common.yaml` gets an `on_error` on `sdo_write`;
  - `8256c1ed26`: no `OD_t`, `CO_SDOclient_init()` gets `nullptr` and the single `0x1280` entry;
  - `9537139499`: `start_sdo_()` returns the `CO_SDO_return_t`, a request that doesn't start logs it at DEBUG
    (`not started (CANopenNode %d)`) before `TRANSPORT_ERROR`; reply tests without the address compare;
    `4acf244406`: reply test state out of globals (clang-tidy).
  bdraco's hardware runs (PR comments, 2026-11-24): an ESP32-C3 client (ESP-IDF) against a `canopen-full` node 0x20
  (ESP32-C3) on a 250 kbit/s bus, a Pico 2 W + MCP2515 as second client (PlatformIO library path), retested at
  `6baffdfcaf`, `06714d7b02`, `5cd20a9dc8` (late responses provoked with slowed objects) and `8256c1ed26`: PASS. Also
  randybb (2026-11-23): two XIAO STM32C5 (esphomeplatformzephyr, `zephyr_can`, 125 kbit/s), PR client against a
  `canopen_entities` server. esphbot's summary (2026-11-24): ready to merge, only the canbus split left; the split is
  esphome/esphome#23247 (`rx_capped()`, merged 2026-11-24, merged into the cut with `dev`), so the cut no longer
  touches canbus. Size: 971 lines outside `tests/` (all added; `git diff --numstat $(git merge-base upstream/dev
  HEAD) -- . ':!tests'`). Tests on the cut: 42 C++.
  Docs (esphome.io#8059, `de283cd69` on 2026-11-24): the 64 KiB limit and late answers by direction; the failed-setup
  rejection was already in `75b0e628a`.
  Ported to `canopen-full` (`534c38bb71`, 2026-11-24), the hub copy here re-synced.
- **Merged** 2026-11-24 as `206bb68485` on `dev` (squash, identical to the cut's tip).

#### PR 2 as cut (2026-11-24)
- **Branch** `canopen-pr2-frames` in `~/repos/esphome`, `20f842f1b3` on `upstream/dev` `206bb68485` (PR 1 merged),
  one commit "[canopen] Add raw frames, heartbeat / EMCY triggers and SDO cancel", pushed to mrk-its. No upstream PR
  yet, no docs yet (esphome.io#8059 is still open: the docs branch would stack on `canopen-pr1-sdo-client`).
- **Built by adding** PR 2's parts to the merged PR 1 tree (not by deleting from `canopen-full`, whose PR 1 parts
  differ: runtime OD, client switch):
  - `frames.cpp` as on `canopen-full`;
  - `types.h`: `NMTState`, `nmt_state_to_string()` (in `canopen.cpp`), `FrameCallback`, `Emergency`,
    `decode_heartbeat()` / `decode_emcy()`;
  - `canopen.h`: `add_frame_listener()`, `send_frame()`, the heartbeat / EMCY callbacks, `cancel_sdo()` and the
    abort state;
  - `driver_can.cpp`: `dispatch_frame_()` after CANopenNode's receive callbacks;
  - `canopen.cpp`: the `init_frame_listeners_()` hook, the abort guard in `idle_()`;
  - `__init__.py`: `on_heartbeat` / `on_emcy` (`CALLBACK_AUTOMATIONS`);
  - `sdo_client.cpp`: `cancel_sdo()`, `abort_sdo_()`, `process_sdo_abort_()`.
  No `USE_CANOPEN_SDO_CLIENT` (PR 3), no `defines.h` lines.
- **Abort limit** (the open point from PR 1's first review): a cancel's abort the bus refuses holds the client's TX
  buffer, so no request could start until the bus came back. It now gets one response timeout to leave
  (`sdo_abort_guard_us_` counts that down while `sdo_abort_pending_`), then it is dropped with a warning and no guard
  follows; once it has left, the 50 ms guard runs as before. Same change on `canopen-full` (`066e724f17`, with the
  tests below; 118 C++ tests).
- **Tests:** `frames_test.cpp` (the decoders + `FramesTest`: listeners before / after setup, RTR / extended frames
  skipped, heartbeat / EMCY callbacks, a listener on an SDO reply, `send_frame()`), `sdo_cancel_test.cpp` (adapted to
  the client-only hub: no `send_nmt()`; plus the abort limit: dropped after the response timeout, no guard after a
  dropped abort; both fail without the limit), `common.yaml` (triggers, `cancel_sdo()`, `send_frame()`,
  `add_frame_listener()`), `test_canopen.py` (the two callbacks in the codegen). 65 C++ + 11 Python tests; builds
  esp8266-ard, esp32-idf, rp2040-ard (no warnings); clang-tidy esp32-idf-tidy (with the tests), esp8266-arduino-tidy,
  bk72xx-tidy, nrf52-tidy clean. Tidy: static test helpers without the trailing `_` (`advance_clock()`).
- **Size:** 262 lines outside `tests/` (+266 / -4), 366 in `tests/`.
- **Upstream:** esphome/esphome#23266 (2026-11-24). Copilot review (2026-11-25), 8 comments, 7 fixed in
  `7ec3825f97` (`canopen-full` `7c2f938315`): `Emergency` by value in the automation (a `std::tuple` can't hold a
  reference), `send_frame()` rejects IDs above `0x7FF`, a filter with `can_id & mask` above `0x7FF` isn't added,
  dispatch by index on copies (a callback may add listeners; `NOLINT(modernize-loop-convert)`), the abort guard holds
  every node (one guard for any number of cancels), the abort's timers run from the cancel (`abort_sdo_()` resets the
  time base), `abort_sdo_()` ends the request before it sends or logs. Pushed back: the guard starts when the driver
  takes the abort, not when it's on the bus (`canbus` has no transmit completion; comment added). 8 new tests (73 C++
  in the cut, 126 on `canopen-full`), each failing without its fix. Docs updated (esphome-docs `67a1f177e`).
  Kōan review: EMCY error reset (code 0) logged at INFO (`784704da37`). Copilot round 2 (2026-11-25), in
  `6fb053048c` (`canopen-full` `78fc5dba07`): `cancel_sdo()` only clears the active request's callback, the abort
  goes out in the next `process_sdo_()` pass (it may run from a log handler inside a transmission owning the TX
  buffer: request sent after its abort, `CANtxCount` wrapped); the guard starts whole in the pass after the driver
  took the abort; `finish_sdo_()` logs after the callback; `init_frame_listeners_()` by index; `send_frame()` takes
  `size_t len`. Pushed back: listener vector on the heap (documented: permanent, add at setup / boot). Tests:
  `FakeCanbus::on_send` (cancel during the first / a retried send), buffer transfers cancelled after the first
  segment; 79 C++ in the cut, 132 on `canopen-full`. Docs `e11375dad`.

#### PR 3 as cut (2026-11-25)
- **Branch** `canopen-pr3-node` in `~/repos/esphome`, `a0b68cae02` on `upstream/dev` `7eb55fbee4` (PR 2 merged),
  one commit "[canopen] Add the CANopen node: node_id, NMT, heartbeat producer, SDO server, identity", pushed to
  mrk-its. Upstream PR esphome/esphome#23299 (2026-11-25).
- **Built by deleting** from `canopen-full` (`f0e18d854d`; its PR 1 / 2 parts now equal `dev`): the canopen
  directories from `canopen-full`, then
  - files of PRs 4-8 deleted: `node_monitor.cpp`, `nmt_master.cpp`, `od_handler.*`, `canopen_node.*`, `pdo.cpp`,
    `pdo_storage.cpp`, `ota/`, and their tests (`canopen_node_test`, `pdo_test`, `storage_test`, `sdo_block_test`,
    `ota/`, `test-ota.*.yaml`, the profile / OTA / block size / `client_only_nmt` configs, `fake_canopen_profile`);
  - lines of PRs 4-8 removed by `pr_split_lines.py --annotate` in `canopen.cpp`, `canopen.h`, `types.h`,
    `__init__.py` (by hand: the `esphome.const` import, which the script tags PR 4 because of `CONF_COMMAND`, the
    schema entries and `_final_validate`'s block size check, `CONF_CANOPEN_ID`); `od.*` without the handler
    extension, `OD_TPDO` / `OD_RPDO`; comments naming later features; includes only later PRs need
    (`preferences.h`, `automation.h`, `<initializer_list>` in `canopen.h`, `<type_traits>` in `od.h`);
  - `to_nmt_state()` `static` again (public only for PR 4's `node_monitor.cpp`); `get_od()` kept (the loopback and
    client-only tests add OD entries with it), `write_od()` / `read_od()` left to PR 5;
  - kept as on `dev` to keep the diff small: `types.h` (`NMTState` in place, one comment changed),
    `sdo_cancel_test.cpp` (bdraco's client-only fixture), the `cancel_sdo()` lambda in `common.yaml`,
    `test_response_timeout_fits_uint16` (all lost on `canopen-full`; ported back in `72308fbd7e`, with `get_od()`
    under its own banner and the script rule "Object dictionary" = PR 3, "OD access" = PR 5);
  - tests trimmed: `od_test` (`Strings`, `Record`), `processing_test` without the RPDO burst, `client_only_test`
    `NoNmt` without `send_nmt()`, `frames_test` without `NodeEventToString`; `no_ota.yaml` renamed `node.yaml`;
    test `__init__.py` enables only the SDO client; `defines.h` adds `USE_CANOPEN_NODE`, `USE_CANOPEN_SDO_CLIENT`,
    `CO_CONFIG_SDO_CLI 0x0B`.
- **Tests:** 108 C++ + 29 Python; builds esp8266-ard, esp32-idf, rp2040-ard (only ESP-IDF's legacy TWAI notice);
  pre-commit. clang-tidy not run (upstream CI runs it on the changed files).
- **Size:** outside `tests/` +887 / -119 (808 non-blank added lines, 656 without comment-only lines; the script
  counts ~550 for PR 3: the rest rewrites PR 1 / 2 code, `canopen.h` regrouped, `receive_()` / loopback in the
  driver, `0x1280` on the runtime OD); `tests/` +938 / -95. Not split further (2026-11-25): the runtime OD alone
  would have one record as its user, the loopback is how the SDO server is tested; identity / device info (~110
  lines) is the clean cut if a reviewer asks.
- **Docs:** esphome-docs branch `canopen-pr3-node` on `canopen-pr2-frames` (`eb98f33a5`): the PR 2 page plus the
  node parts, worded after `feature/canopen-sdo-client-only` (`bae1c36ff`; the esphome-docs `canopen-full` page still
  has `node_id` required): intro, `node_id` / `heartbeat_interval` / `sdo_client` / `nmt_startup` / identity /
  device info / `on_nmt_state_change`, SDO Client Only, Several Nodes, Object Dictionary (PR 3's objects), NMT State
  of This Node, requests to the own id. First reviewed as the fork PR mrk-its/esphome-docs#1 (base
  `canopen-pr2-frames`); after #8243 was merged, rebased on `next` (clean; `30bb7c4f7`, the pre-rebase branch is
  `backup/canopen-pr3-node-pre-rebase`) and opened as esphome/esphome.io#8254 (2026-11-25); the fork PR is closed.
- **Copilot review** (2026-11-25, 14 comments on `a0b68cae02`), all answered in the threads, plus a summary comment:
  - fixed: `CONF_HARDWARE_VERSION` from `esphome/components/const` (esphome/esphome#23300, merged; the fork got the
    constant too, uncommitted), `canopennode_path` dropped (`dfbdca9ec1`); node members and setters only with
    `USE_CANOPEN_NODE`, `0x1014` / `0x1280` read only (`e3f9c80f02`); `dump_config()` after a failed setup, one flush
    per `loop()` (`in_frame_` -> `hold_flush_`, also around `process_node_()`), no SDO in STOPPED (requests end with
    `SDO_RESULT_REJECTED`; `6b71642da4`); `heartbeat_interval` at least 100 ms (`33eda18fd6`). New tests:
    `LocalPair.ReadOnlyCommunicationObjects`, `CanopenProcessing.SdoTimeoutCountsFromTheFlushThatSentTheRequest`,
    `CanopenProcessing.StoppedNodeRejectsSdoRequests`, `test_heartbeat_interval_range`; each fails without its fix.
  - not changed, explained: local nodes' SDO clients share a server's channel (documented under Shared SDO Channel),
    the boot-up message replaced by a heartbeat on a bus that refuses frames for over 500 ms (CANopenNode's producer),
    NMT Reset Communication only logged (its own PR), local delivery overwriting back-to-back frames (with PRs 4 / 6).
  - Docs: `0x1014` / `0x1280` read only, `heartbeat_interval` from 100 ms, no SDO in STOPPED, the shared channel between
    `canopen` components (esphome-docs `canopen-pr3-node` up to `b0fc465aa`).
  - The docs' "the node doesn't send EMCY frames" is corrected (`7d0c6b1fd`): EMCY only for errors the stack detects
    (with our driver only a full TX buffer reaches the producer), no API for own messages.
- **bdraco's commits** (2026-11-25, on top of `33eda18fd6`, head `fe354804ff`; CI green):
  - `a32c32089f`: one `setup()` for both modes (`setup_client_only_()` gone, node parts under `if (node)`),
    `to_nmt_state()` a cast with `static_assert`s on CANopenNode's values, `rx_capped()` called directly (no
    compatibility template), `receive_()` returns whether a CANopenNode object took the frame and only then the node
    processes it at once, `add_optional` in the codegen.
  - `7489a8fdb3`: `add_*()` collect into a temporary `Pending` block, `finalize()` copies it into exact-size
    `FixedVector`s and frees it (`shrink_to_fit()` doesn't free everywhere); `finalized()` is `od_.list != nullptr`.
  - `fe354804ff`: members ordered by size (about 16 bytes of padding less per node hub).
  - Ported to `canopen-full` (`8fbc2a199b`, `34ac7fb9dc`, `c9a2eaef3c`, cherry-picked): the PDO / storage and node
    monitor hooks go into the `if (node)` blocks; `to_nmt_state()` stays public (`node_monitor.cpp`); the handler
    extensions (PR 5) are collected in `Pending` too and copied into their own `FixedVector`; the node monitor members
    grouped by size as well. Afterwards `driver_can.cpp`, `sdo_client.cpp`, `device_info.cpp`, `frames.cpp` and
    `automation.h` are identical on both branches, `od.*` / `canopen.*` differ only by PRs 4-8. The direct
    `rx_capped()` needs the fork stub (Platform notes in CLAUDE.md).
- **Copilot's second round** (2026-11-25, 8 comments on `fe354804ff`), all resolved by bdraco: fixed in his commits
  below (A: the flush test fed a frame no CANopenNode object takes, B: `memcpy` from `nullptr` for an empty local
  frame, C: a local transfer that ended early sent an SDO abort on the bus); declined with reasons in the threads (D:
  STOPPED drops the late-answer guard, E: a local `set_nmt_state(STOPPED)` applies one loop late, F: the capped-RX gate
  delays STOPPED cleanup and own-id transfers, G: `0x1017` writable below 100 ms over SDO, H: more build fixtures).
  esphbot's suggestion of a final-validate warning for shared SDO channels: declined in a PR comment (the config can't
  tell whether two components ever talk to one node at the same time).
- **bdraco's second batch** (2026-11-25, head `9e7687e388`): `3ef32bc17c` local transfers compiled only with a node
  (`CO_CONFIG_SDO_CLI` `0x03` without any `node_id`, about 750 B), an empty local frame skips the copy, the flush test
  feeds an NMT command; `c0e9ee180f` local transfers end without an abort on the bus or a guard (`sdo_is_local_()`),
  a client-only config drops the node options' defaults (its validated config validates again), tests for own-id
  reads and NMT commands; `9e7687e388` `set_heartbeat_interval(uint16_t)`, the "late calls" comment narrowed to OD
  entries. Ported to `canopen-full` (`1e59f83406`, `8d17ef3d5d`, `e8c940ed02`; one conflict, the class comment, which
  there lists what `canopen-full` checks: OD entries and handlers, PDO configurations, monitored nodes).
- **bdraco's hardware tests** (two ESP32-C3 boards, 250 kbit/s; comments on the PR): `fe354804ff` passed with caveats,
  `9e7687e388` passed. Client-only firmware is +3,020 B flash / +8 B RAM over `dev` (the runtime OD builder; was
  +3,992 B before `3ef32bc17c`); heartbeat period = interval + up to one loop pass (1,008 ms for 1 s);
  `get_nmt_state()` is `UNKNOWN` until the first loop pass. PR description and docs updated to match (size claim,
  implementation notes, test counts 113 C++ / 31 Python; docs `7d0c6b1fd`).
- **PR 2 docs** (#8243): Kōan's round (2026-11-25) in `4354f6b56` (TPDO1 comment `0x180`-`0x1FF`, "The 50 ms are
  counted"), description updated (listeners can be added at any time). Merged 2026-11-25 (squash `67bb87134` on
  `next`), after bdraco merged `next` into the branch and added a paragraph on the frame listener's plain-function
  callback and `context` lifetime (`26969462c`).

#### PR 4 as cut (2026-11-26, esphome/esphome#23327)
- **Why now:** PR 3 (#23299) is close to merge; PR 4 is prepared so it can go up right after (a PR opened before would
  carry PR 3's commits). Two items deferred in PR 3's review land here.
- **On `canopen-full`** (pushed), on top of `e8c940ed02`:
  - `02454e52f3` frames between local nodes go through a per-receiver queue: `FixedRingBuffer<LocalFrame, 16>`
    (`local_queue_`, allocated in `setup()` only with local peers), filled by the sender's `transmit()`
    (`queue_local_()`, no logging), emptied at the start of the receiver's `loop()` (`deliver_local_()`), each frame
    through `handle_frame_()` (shared with `on_frame_()`: `receive_()`, SDO reply, `process_node_()` if taken). Full
    queue: frame lost, counted (`local_dropped_`), logged from `loop()`. Fixes Copilot's PR 3 points (back-to-back
    frames to one CANopenNode object overwrite each other; a relayed frame overtaking its cause). A peer now sees a
    local frame in its own next `loop()` (3 loopback tests adapted). New tests: `BackToBackFramesToOneObjectAreKept`,
    `CanopenLocal.FramesKeepBusOrder`, `FullQueueLosesFrames` (the first two fail with direct delivery).
  - `abe8b31b3a` heartbeat consumer only when used: `enable_node_monitor()` (for `heartbeat_clients` and
    `register_node(heartbeat_timeout=...)`) defines `USE_CANOPEN_NODE_MONITOR`; otherwise a `CoroPriority.FINAL` job
    (`_final_build_flags`, scheduled once from `to_code()`) adds `-DCO_CONFIG_HB_CONS=0`. `monitor_node()`, 0x1016, the
    consumer callbacks are under the define; `get_node_state()` / `is_node_online()` fall back to UNKNOWN / false;
    `node_event_callback_` stays (the `CANopenNode` profile base uses it). ESP8266 node without monitoring: 0
    `CO_HBconsumer` symbols, 2,804 B less flash (checked on builds with and without). Codegen test
    `test_node_monitor_only_when_used`; unit-test override calls `enable_node_monitor()`; `defines.h` has the define.
  - `8368ad29d9` `tests/components/canopen/node_monitor_test.cpp`: node events (online with state, NMT changed, reset),
    offline after a 30 ms timeout (real clock: `process_node_()` uses `micros()`, the test sleeps), 0x1016 entries
    (shortest timeout, nothing after setup), NMT master commands (other node, own id, all nodes, invalid id),
    client-only refusing. 160 C++ tests pass on `canopen-full`.
- **Cut** `canopen-pr4-monitor-nmt` (`c723acee71`, one commit "[canopen] Add node monitoring (heartbeat_clients,
  on_node_event) and the NMT master (canopen.nmt)", pushed to mrk-its/esphome): `canopen-full`'s canopen dirs minus PRs 5-8
  (files deleted, lines by `--annotate` >= 5), `od.*` / `od_test` / `processing_test` / `node.yaml` from the PR 3 cut,
  the class comment = PR 3's plus PR 4's parts, PR 3-style comments restored (no `OdObject` / PDO mentions, no
  `automation.h` / `preferences.h` / `<initializer_list>` includes), `CONF_CANOPEN_ID` dropped, `sdo_block_size`
  removed as in PR 3, `test_canopen.py` = PR 3's plus PR 4's tests (lambda id back to `_5`), `defines.h` +
  `USE_CANOPEN_NMT_MASTER`, `USE_CANOPEN_NODE_MONITOR`, `CO_CONFIG_NMT 0x03`. Size: +411 / -24 code, +405 tests;
  123 C++ + 38 Python tests pass. Cross-checked: vs `canopen-full` only PRs 5-8 differ (driver_can, nmt_master,
  node_monitor, frames, sdo_client, device_info identical); vs the PR 3 cut only PR 4 parts.
- **Checks (2026-11-26), all green:** the cut builds esp8266-ard / esp32-idf / rp2040-ard
  (`test_build_components.py`, no canopen warnings), client-only and no-monitoring configs (`client_only.yaml`,
  `node_nomon.yaml`, ESP8266); esphome-canopen with the hub synced to `canopen-full` `8368ad29d9` (committed,
  `441daab`): pytest (97), esp8266 mapper, `stm32l4_canopen`. clang-tidy on the cut clean: esp32-idf-tidy on the
  component + tests (18 files), esp8266-arduino-tidy, bk72xx-tidy, nrf52-tidy on the component (8 files). The `/tmp`
  tmpfs quota broke tidy (libdeps in the worktree's `.pio`) and builds: run them with
  `PLATFORMIO_WORKSPACE_DIR=~/.cache/esphome-tidy-pio/<wt>` and `pio` on `PATH` (the venv's `bin`), builds outside
  `/tmp`.
- **Docs cut** on esphome-docs `canopen-pr4-monitor-nmt` (`90d041b15`, on `canopen-pr3-node` `e3044c456`, pushed to
  mrk-its/esphome-docs): `canopen.mdx` +115 / -5: intro bullets, `heartbeat_clients` / `on_node_event` options, "Heartbeat
  Monitoring" (events checked against CANopenNode's consumer: no timeout before the first heartbeat, offline again
  after a boot-up until the next heartbeat; logging; an `on_node_event` example re-configuring a restarted node; the
  `is_node_online()` / `get_node_state()` lambdas, UNKNOWN / false when not monitored), `0x1016` in the OD table,
  the stack's EMCY on a consumer timeout, `on_node_event` trigger, `canopen.nmt` action (applies to us for our id or
  0; rejected without `node_id`), "NMT Master" lambda (`send_nmt()` false only while the previous command still waits:
  `CO_CANsend()` queues), "Several Nodes": monitoring / NMT between local nodes, the local queue (next loop pass, bus
  order, 16 frames, the rest lost + warning). `node script/lint.mjs` passes; the examples pass `esphome config` and
  compile (esp32-idf, `PYTHONPATH` = the PR 4 worktree).
- **Rebased and opened (2026-11-26):** #23299 merged (squash `e187cf0e5d`, canopen tree identical to its head
  `9e7687e388`; no other `dev` commit touched canopen / canbus / const). `git rebase --onto upstream/dev
  origin/canopen-pr3-node`: clean, `3f4b44effe` on `dev` `6f7cd90870`, `range-diff` identical to `c723acee71`;
  force-pushed to mrk-its. Re-run: 123 C++ + 38 Python tests, `esphome config` and builds esp8266-ard / esp32-idf /
  rp2040-ard (only ESP-IDF's legacy TWAI notice); clang-tidy not re-run (same code). esphome-canopen pytest (97)
  with `../esphome/venv`. Opened as esphome/esphome#23327 without the docs link.
- **Review (2026-11-26):** Copilot (11 comments) and esphbot (changes requested; its points overlap Copilot's, plus
  `on_node_event` without `heartbeat_clients` never firing). Fixed (pushed to mrk-its 2026-11-26: `canopen-full` `67611e6c23`, cut `4f80eb3f9c`; hub synced here): Copilot #4
  (`queue_local_()` uses `enable_loop_soon_any_context()`: `enable_loop()` logs at VV and is meant for the component's
  own context) = `canopen-full` `84546abfad`, cut `c86306677f`; Copilot #3, #5a, #6, #7, #8, #9, #10 =
  `canopen-full` `edbb935c3f`, cut `2eee1b96d9`: `send_nmt()` refuses (and logs) a command while `NMT_TXbuff` is full
  or, for us / all, while `internalCommand` is pending, logs refused / failed commands; 0x1016 `ODA_SDO_R`;
  `monitor_node()` ignores id 0 / > 127 / own / timeout 0; `init_node_monitor_()` sets
  `HBcons->NMTisPreOrOperationalPrev = true` (the first `CO_process()` otherwise clears a heartbeat / boot-up received
  before it); `on_frame_()` calls `deliver_local_()` first. 5 new tests, each fails without its fix (the busy-buffer
  one reboots: the "rejected" broadcast reset_node got applied). 165 / 128 C++ + 38 Python tests pass; no builds or
  clang-tidy yet. Copilot #5b = `canopen-full` `8fad14a320`, cut `07ab44bad3` (conflict with the PDO lines resolved):
  on `CO_RESET_APP` `process_node_()` sets `reset_pending_` and reboots once `CANtxCount == 0` (loop() flushes before
  it processes the node) or after `RESET_FLUSH_TIMEOUT_MS` (100); test `ResetNodeRebootsAfterItsFrame` runs each case
  in an `EXPECT_EXIT` child (the host's `safe_reboot()` is `exit(0)`, which the runner would take as a pass), fails
  without the fix. 166 / 129 C++ tests pass. esphbot 8 = `canopen-full` `67611e6c23`, cut `4f80eb3f9c`: warning
  "canopen <id>: on_node_event never fires, no node is monitored" from `_final_build_flags` (FINAL, so components
  enabling the monitor count), test `test_node_event_without_monitored_nodes` (config `node_event_unmonitored.yaml`);
  49 / 40 Python tests pass. Commits in the cut worktree need `../esphome/venv/bin` on `PATH` (pre-commit's pylint).
  Replies posted and the 8 fixed Copilot threads resolved; esphbot answered in a PR comment. Copilot #2 / esphbot 3 =
  `canopen-full` `8cf24cb83f`, cut `eff62bc872` (pushed): `nmt_master: true` (a node option, `NODE_OPTIONS`; `false`
  = unset) calls `enable_nmt_master()`; tests `test_nmt_master_true`, `test_nmt_master_only_when_used`, client-only
  rejection; a lambda-only `send_nmt()` builds on ESP8266 with it and fails without. Docs: esphome-docs
  `canopen-pr4-monitor-nmt` `93bdec276` (pushed over SSH: `origin` there is HTTPS without credentials): `nmt_master`,
  0x1016 read only, `send_nmt()` false cases, `reset_node` to all restarts us once sent. #2 thread replied and resolved,
  esphbot 3 answered in a PR comment. Still open: #11 (see "Next" below); #1 answered without a code change.
- **State at the end of 2026-11-26 (handoff to another machine).** Everything is committed and pushed:
  - mrk-its/esphome `canopen-full` = `ff573b5fe0`; `canopen-pr4-monitor-nmt` (= #23327's head) = `7cc83fcc04`
    (`3f4b44effe` + 10 review-fix commits; upstream squashes). Size: +588 / -31 code, +800 / -4 tests. #23327 CI after `eff62bc872`: 79 checks pass, 7 skipped
    (builds and clang-tidy included); not checked after `b777e48bcd`.
  - mrk-its/esphome-docs `canopen-pr4-monitor-nmt` = `ab09b70f7` (on `canopen-pr3-node`; + "NMT commands for this node
    apply right away", + "Local nodes keep the bus order with frames from the bus"); the docs PR isn't opened.
  - esphome-canopen `canopen_entities`: hub copy synced from `canopen-full` `ff573b5fe0` (`test_hub_copy.py` passes).
  - Review: 9 of Copilot's 11 threads replied and resolved; esphbot's points answered in three PR comments
    (#issuecomment-7198868548, -7199214927, -7203675630 = point 6, `uint8_t` kept). No human review yet. #1 answered and resolved, #11 fixed and resolved (see below).
  - Copilot round 2 (on `eff62bc872`, 5 comments). Done: 4712656148 (`CANtxCount == 0` means the driver accepted the
    frame, not that it is on the bus; the `canbus` API can't tell): `canopen-full` `d805648f7c`, cut `b777e48bcd`,
    the reboot also waits `RESET_TX_GRACE_MS` (20) after CANopenNode's queue is empty, still capped at 100 ms
    (`reset_flushed_` / `reset_flushed_ms_`); the test checks no reboot 19 ms after the hand-over, a reboot at 20 ms
    (fails on the old code); replied (mentioning a possible `canbus` `tx_pending()` PR for exact completion) and
    resolved. 4712656233 (`nmt_master.cpp:12`, = #1 again): answered with #1's reply and resolved.
    - 4712656270 (`nmt_master.cpp:24`, a command for us overwritten by a received NMT frame or `set_nmt_state()`
      before `CO_process()`) and 4712656299 (`nmt_master.cpp:29`, `send_nmt()` re-entered from a log handler inside
      its own send, `CANtxCount` counted twice): **done (2026-11-26)**, `canopen-full` `023b09c800`, cut `72a14b41f7`.
      Commands for us (`send_nmt()` to our id / 0, `set_nmt_state()`) are applied right away (`apply_nmt_command_()`
      runs `process_node_()`, keeping `hold_flush_`); from an automation the processing runs (flag `processing_`, set
      inside `process_node_()`) they wait for the next processing, and `set_nmt_state()` is refused while one waits.
      `nmt_sending_` around `CO_NMT_sendCommand()` refuses a nested `send_nmt()` (no logging); `CO_CANsend()` counts a
      queued buffer once. Tests `NmtForUsTest.*` (4) and `CanopenNmtMaster.CommandFromInsideItsOwnSend`, all fail on the
      old code (`SecondCommandForUsBeforeProcessing` replaced); 171 / 134 C++ + 52 / 43 Python pass; no builds /
      clang-tidy. Docs updated (`set_nmt_state()` / `send_nmt()` comments). Replied and resolved.
    - 4712656195 (`driver_can.cpp:37`, a reply A sends while handling a bus frame reached local node B before that bus
      frame): **done (2026-11-26)**, `canopen-full` `ff573b5fe0`, cut `7cc83fcc04` ("option B"). The local node whose
      canbus callback runs first for a bus frame queues it for the other local nodes (`stage_bus_frame_()`, sets their
      `bus_frame_staged_`) before it handles it; their callbacks then only drain their queue. Relies on
      `Canbus::loop()` calling all callbacks for a frame before reading the next, and on the queue being allocated
      right before the callback is registered. The flag is set even on a full queue (no double staging); a bus frame
      on a full queue is lost for that node. Tests `LocalPair.ReplyAfterTheBusFrame`, `LocalPair.BusFrameOnAFullQueue`
      (both fail on the old code; the second because bus frames were never dropped before);
      `LocalFramesBeforeALaterBusFrame` now goes through `bus.loop()`. 173 / 136 C++ + 52 / 43 Python pass; no builds
      / clang-tidy. Docs ("Several Nodes") updated. Replied and resolved: no open review threads left.
- **Copilot #1** (thread 4705205472, = esphbot 6; and round 2's 4712656233): a templated `canopen.nmt` node_id is cut
  to `uint8_t`, so 256 becomes 0 = all nodes. **Decided (2026-11-26): no code change.** `uint8_t` is the node-id type
  across the hub's API (`read_sdo()`, `write_sdo()`, `monitor_node()`, the SDO actions); node id 0 stays a valid
  broadcast target; static `node_id:` values are schema-checked (0-127); a lambda returning a wider value narrows like
  any other `uint8_t` argument (GCC's `-Woverflow` warns for constants; runtime values only with `-Wconversion`, not
  enabled). Widening only `send_nmt()` would be inconsistent. Replied on both threads with that and resolved them.
- **Copilot #11** (thread 4705205903), **done (2026-11-26)**: CANopenNode's consumer calls the started / remote-reset /
  timeout callbacks before it stores `HBstate` / `NMTstate`, so an `on_node_event` automation querying
  `is_node_online()` / `get_node_state()` saw the old state. `canopen-full` `7930c40376`, cut `4b0926f608` (pushed): the
  `on_hb_*` callbacks only push `{node_id, event, state}` into `FixedVector<PendingNodeEvent> pending_node_events_`
  (`init(2 * hb_cons_.size())` in `init_node_monitor_()`), `process_node_()` calls `dispatch_node_events_()` after
  `CO_process()`. Capacity: one consumer pass fires at most 2 callbacks per node (frame or timeout, then NMT changed; a
  frame and a timeout exclude each other in one pass; more frames between passes collapse, `CANrxNew` is a flag), the
  queue is drained after every `CO_process()`, the node set is fixed after setup; `push_back()` ignores overflow anyway.
  Test `CanopenNodeMonitor.QueriesInAnEventSeeTheNewState` (fails on all 6 checks without the fix). 167 / 130 C++ +
  52 / 43 Python tests pass; no builds / clang-tidy. Replied (with the capacity reasoning) and resolved. Side note: an
  OFFLINE is followed by NMT_CHANGED(UNKNOWN) and a RESET by NMT_CHANGED(INITIALIZING) in the same pass (the consumer's
  NMT-changed check after the timeout / boot-up); unchanged by this fix, not checked against the docs yet.
- **Next: docs.** (1) Once esphome/esphome.io#8254 (PR 3's docs) is merged: rebase the docs branch onto `next`
  (`git rebase --onto <esphome.io next> canopen-pr3-node`; `../esphome-docs` has only the mrk-its remote, add
  `esphome/esphome.io` to fetch `next`), open the docs PR (the user may need to run `gh pr create` for esphome.io: the
  permission check blocked it once), link it in #23327's description. (2) Port the docs back to esphome-docs
  `canopen-full` (diverged, still the pre-rebase text), including `93bdec276`. (3) Any fix of #1 / #11 that changes
  behaviour goes into the docs branch too.
- **Flow for each fix** (as done for the review): commit on `canopen-full` first, run `script/cpp_unit_test.py canopen`
  and `pytest tests/component_tests/canopen` there, check the new test fails without the fix; cherry-pick onto
  `canopen-pr4-monitor-nmt` (conflicts are lines of PRs 5-8: PDO, `sdo_block_size`, `register_node`, OTA; the cut's
  plain node config is `node.yaml`, not `no_ota.yaml`), run the tests there; push both to mrk-its; `scripts/sync_hub.py`
  here, `pytest test/`, commit and push; reply on the thread and resolve it (GraphQL `resolveReviewThread`).
- **Setting up a machine** (sibling checkouts of esphome-canopen): `../esphome` (mrk-its/esphome, remote `upstream` =
  esphome/esphome; `script/setup` makes `../esphome/venv`, an editable install of that checkout), `../esphome-docs`
  (mrk-its/esphome-docs), `../CANopenNode` (the fork, for reading the library). Worktrees for the branches, e.g.
  `git -C ../esphome worktree add --track -b canopen-full <dir>/full-wt origin/canopen-full` and the same for
  `canopen-pr4-monitor-nmt`; esphome-docs `canopen-pr4-monitor-nmt` likewise (`npm ci` in that worktree before
  `node script/lint.mjs`). Run the fork's tests with `PYTHONPATH=<worktree> ../esphome/venv/bin/python
  script/cpp_unit_test.py canopen` (and `-m pytest tests/component_tests/canopen`); for commits there put
  `../esphome/venv/bin` on `PATH` (pre-commit's pylint and clang-format). The unit-test binary ignores
  `--gtest_filter`; use `GTEST_FILTER=... <build>/.pioenvs/cpptests-*/program`. Builds / clang-tidy outside `/tmp`
  (`PLATFORMIO_WORKSPACE_DIR=~/.cache/esphome-tidy-pio/<wt>`). Pushing esphome-docs over HTTPS failed without
  credentials here: push to `git@github.com:mrk-its/esphome-docs.git`.

### Possible reorder: node monitoring without the node core (not planned, 2026-11-07)
An option for later, not the current plan: replace CANopenNode's heartbeat consumer with our own monitor on PR 2's
decoded heartbeat frames, so node monitoring works on a hub without `node_id` and can come right after PR 2.

**Why it came up.** kj4dxk's `cia402` port (fork `kj4dxk/esphome`, branch `cia402-canopennode`, `52dd4d0222`, one
commit on `canopen-full` `613abd0cf7`, 1186 lines; its only hub change is the `<CANopen.h>` include, already on
`canopen-full`) needs more than PRs 1 + 2:

| cia402 uses | PR |
|---|---|
| `read_sdo()` / `write_sdo()` / `cancel_sdo()` (all drive steps) | 1 |
| `on_node_emcy()` (fails a running command), `Emergency`, `NMTState` | 2 |
| `is_operational()` (`is_node_online()` + `get_node_state()`), checked in `begin_()`: without monitoring every command is `DRIVE_RESULT_REJECTED` | 4 |
| `on_node_offline()` / `on_node_reset()` / `on_node_nmt_changed()`, `heartbeat_timeout` (default 1500 ms) | 4 |
| `CANopenNode`, `register_node()`, `final_validate_canopen_node()` | 5 |
| a hub with `node_id` (`final_validate_canopen_node()` -> `require_node_id()`) | 3 |
| `register_node(..., nmt=True)`: compiles the NMT master, but cia402 never calls `send_nmt()` | (4, unused) |

The hard dependency is monitoring (PR 4), which needs the node core (PR 3): CANopenNode's HB consumer reports
timeouts through the EM object, and `CO_EM_init()` rejects node id 0.

**Design (one implementation, not a second one next to the HB consumer).** `node_monitor.cpp` drops the
CANopenNode glue (`0x1016`, `CNT_HB_CONS`, the four `on_hb_*` callbacks) and keeps its API: `monitor_node()`,
`is_node_online()`, `get_node_state()`, `add_on_node_event_callback()`, `heartbeat_clients`, `on_node_event`.
- Monitored nodes in a fixed list filled before setup (node id, timeout, last state, time left).
- From PR 2's frame path (`decode_heartbeat()`): boot-up (state 0) -> `RESET`; first heartbeat (or the first after a
  timeout) -> `ONLINE`, then `NMT_CHANGED` (unknown -> state); another state -> `NMT_CHANGED`; every heartbeat
  restarts the node's timer.
- Timeouts counted down with the hub's `dt` in `process_()` -> `OFFLINE`, then `NMT_CHANGED` (state -> unknown).
  Same event order as today, so `CANopenNode` and its users don't change.
- Lost: OD `0x1016` (optional in CiA 301; the node core could add it back as a record whose handler sets the
  timeouts) and CANopenNode's EMCY on a consumer timeout (the hub sends no EMCY anyway). Gained: works without
  `node_id`, `CO_CONFIG_HB_CONS` not compiled, unit tests with `TestHub::frame()`.
- About the size of today's glue (~100 lines + tests).

**Resulting order:** 1 SDO client -> 2 raw frames -> 3 node monitoring + profile API (`CANopenNode`,
`register_node()` without `require_node_id()`, which stays with the node core) -> cia402 (kj4dxk; drop
`nmt=True`) -> node core, NMT master, OD API, PDOs, block transfer, OTA. `send_nmt()` could also move before the node
core (an NMT command is one frame to `0x000`, `send_frame()` is enough), but nothing needs it there yet. This is close
to the feature set of kj4dxk's #21522 ("CANopen client with SDO, NMT and node events").

### Old split (superseded)
Kept for the history of what each part contained; the branches below are not used any more.

Old split (revised 2026-10-29: the first version of PR 1, ~1050 production lines, was too big for a first PR):

| PR | Content | Lines |
|---|---|---|
| 1. node + heartbeat monitoring | CANopenNode integration (library, port header trimmed to NMT / HB producer+consumer / EM, driver, runtime OD with VAR/ARRAY), NMT state machine, heartbeat producer, `heartbeat_clients` + `on_node_event`, `is_node_online()` / `get_node_state()` | 845 (measured, incl. comments; tests 175) |
| 2. raw frames and events | `on_heartbeat` / `on_emcy` (+ decoders, `Emergency`), `add_frame_listener()`, `send_frame()` | ~150 |
| 3. SDO client + NMT master | `sdo_client.cpp`, `add_record()` (0x1280), `canopen.sdo_write`, `canopen.nmt`, optional parts | ~450 |
| 4. profile API | `canopen_node.h` (`CANopenNode` base class, hooks, forwarders), `register_node()`; branch `canopen-4-profile` (deleted 2026-11-01), done ([cia402.md](cia402.md), "PR 4") | 145 (tests ~240) |
| 5. `cia402`, SDO | direct port of #21533 onto the hub (ideally by its author) | ~450 |
| 6. PDO process data | `add_pdo_listener()` / `send_pdo()`, `SDOSequence`, `configure_rpdo/tpdo`; `cia402` `process_data: pdo` | ~450 |

The SDO server and our own entities/OTA/logs (steps 3-8) are a later chain.

Branches, tests and results of PRs 1-3: [upstream_prs_1-3.md](upstream_prs_1-3.md); PR 4 (`canopen-4-profile`,
deleted 2026-11-01): [cia402.md](cia402.md), "PR 4". Each branch is cut from
`components_next/canopen` (the full, hardware-tested tree) in `~/repos/esphome`; later branches stack on it.

### Before PR 1 (applies to the new split too)
- **Library source:** no `file://` path. Stock CANopenNode can't be used as is: it has no `library.json`, and the
  Zephyr and native ESP-IDF backends of ESPHome require a manifest (`esphome/platformio/library.py`, only the `host`
  backend accepts manifest-less libraries; plain PlatformIO builds would work). Done (2026-10-29): packaging-only
  fork (branch `esphome`, upstream + `library.json`, pushed), pulled from GitHub by commit; since 2026-11-01 the
  port header is in the fork too (see "Port header"), so the fork stays even if upstream takes the `library.json`
  (unless the header moves back once upstream's clang-tidy can reach a component's include dir). The block-size
  patch (branch `sdo-srv-blksize`) is needed only from step 3 on (drop it, or send it
  upstream).
- **ESP32 / ESP8266 builds** (upstream CI: esp32-idf, esp32-ard, esp8266): all three compile with upstream `dev`
  (`test/esp32_canopen.yaml`, `test/esp8266_canopen.yaml`). `driver_can.cpp` uses `send_data(..., data, len)`,
  which is in `dev` but not in the 2026.9.x releases, so `components_next` needs ESPHome newer than 2026.9 (fine for
  PRs against `dev`; matters only if it ships as an external component before the next release). Run on real ESP32
  hardware: done (2026-11-01, see "Current state", "ESP32 hardware").
- **API churn first:** the rename and `heartbeat_clients` rebuilt on node monitoring go into PR 1, so PR 2 doesn't
  rewrite code under review.
- **Upstream boilerplate:** `tests/components/canopen/*.yaml`, Python tests, CODEOWNERS, esphome.io docs, the
  clang-tidy setup (`platformio.ini` `lib_deps_base`, `defines.h`; see "Cutting the branches", step 2). The
  CANopenNode pin is then in two places (`CANOPENNODE_SOURCE`, `platformio.ini`): bump both.
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
- clang-tidy (23.1.4, `script/clang-tidy -e <env> esphome/components/canopen`) is clean on esp8266-arduino-tidy,
  esp32-idf-tidy, bk72xx-tidy and nrf52-tidy (2026-11-01, `canopen-full` `5e2a6cf`; it found two issues, fixed there:
  identical SDO read `case`s, a `uint8_t` loop index against `size()`). Its setup is committed on `canopen-full` (`62bd875`)
  and is cut into the PRs like the component ("Cutting the branches", step 2): CANopenNode (the pinned
  GitHub URL; esp32-idf-tidy rejects `file://` and `git+file://`) in `platformio.ini` `[common] lib_deps_base` (the
  only list every tidy env gets; esp32-idf-tidy reads only the `[common*]` sections), and `USE_CANOPEN_*`,
  `CANOPEN_SDO_BLOCK_SIZE` and the `CO_CONFIG_*` values in `esphome/core/defines.h` (`od.h` / `od_handler.h` include
  it before `CANopen.h`; esp32-idf-tidy ignores `build_flags`, so they can't go there). No include path is needed since
  the port header is in the CANopenNode fork (see "Port header").
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
    (`CANOPEN_ENTITIES_ENTITIES`), states in `std::array`, callbacks capturing only the entity pointer,
    `static constexpr`, `ESPHOME_LOG_TAG`, concatenated namespace, Python type hints. stm32l4 flash -2.8 KB.

## Hub for the entity mapper
Target split (2026-10-30): the `canopen` hub (generic CiA 301) goes upstream; everything specific to esphome-canopen
and can2mqtt stays here as the external component `canopen_entities` (named `esphome_entity_mapper` until
2026-10-31, then `canopen_entity_mapper` until 2026-11-01), which refers to a hub with `canopen_id`:
entities (0x2000+16n), TPDO layout and hash (0x3002), remote states (0x4000+n, `rpdo: {node, entity}`,
`platform: canopen_entities`, `canopen_entities.<domain>.<action>`), the OD writer, OTA (0x3000), remote logs (0x3100), and the
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
   survive a reboot. **Done, PR 10** (2026-10-31, `canopen-next`): CANopenNode's `CO_storage` (`CO_CONFIG_STORAGE`
   from `enable_pdo()`) with one entry, the PDO parameters (RPDO and TPDO records), on 0x1010 / 0x1011 sub 1 (all)
   and 2 (communication); only with PDOs and preferences. "save" stores the records as they are (`PdoParams` vectors
   as bytes) plus a validity marker, then `sync()`; boot loads them before the PDO init (into copies: a failed load
   keeps the defaults); "load" clears only the marker (the defaults apply from the next boot, CiA 301; the old
   component's `global_preferences->reset()` wiped every component's preferences). The preference key hashes the
   codegen defaults and the node id, so a firmware with other PDO defaults ignores a stored set. All platforms (the
   old component: ESP32 only). Old saved configs (`canopen_comm_state_v2`) are not migrated: the host saves again.
   Tests: gtest `tests/components/canopen/storage_test.cpp` (save, reboot, wrong signature, restore, sub 1, other
   defaults); builds ESP8266, ESP32-C3 (esp-idf), Zephyr; nucleo: save survives a reset, restore brings the defaults
   back after one (TPDO 6 event timer, temporary test interval).
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
   nodes on the same canbus; was step 8). **Done, PR 11** (2026-10-31, `canopen-next`): codegen links the nodes of
   each canbus into a ring (`set_next_local()`, config order; one pointer per node, no globals); validation rejects
   two nodes with the same `node_id` on one canbus. `transmit()` (CANopenNode's driver and `send_frame()`) sends on
   the canbus and, only once the platform has taken the frame, hands it to the other nodes' `receive_()`:
   CANopenNode RX buffers, frame listeners, heartbeat / EMCY callbacks, **without** `process_()`, which runs in their
   next `loop()`. So no node's CANopenNode processing runs inside another's (a reply can't arrive inside the
   sender's `CO_CANsend`), at the cost of one loop per hop for local SDO. A frame queued in a TX buffer (platform
   busy) is delivered when `CO_CANmodule_process` resends it. Not covered: a frame listener that sends to a local
   node, which sends back from its listener, recurses (user loops only). gtest `loopback_test.cpp` (boot-up seen by
   the other node only, frame listener / `send_frame()`, a refused frame not delivered, SDO upload of 0x1018:1 from
   the other node over the fake bus, a queued frame delivered when sent); Python: the ring per canbus, the duplicate
   id. Builds ESP8266 (mapper), ESP32-C3 (esp-idf), Zephyr. Nucleo (2026-10-31, `test/stm32l4_loopback.yaml`: nodes
   0x7e and 0x7d on the one canbus): each monitors the other (boot-up, online, operational), node 0x01 from the real
   bus reaches both, expedited SDO reads both ways, a write from b to a (read back by a locally), a block upload of
   a's 0x1008 by b (segmented, a without `sdo_block_size`), `send_frame()` from a to b's frame listener (a doesn't
   get its own frame).
   **SDO client fix found by it** (2026-10-31, `canopen-next`, to fold into PR 3): `read_sdo_data()` asked for a
   block upload and `write_sdo_data()` for a block download above 32 bytes; a server without block transfer
   (optional in CiA 301: the hub without `sdo_block_size`, many third-party devices) aborts that with 0x05040001
   (`CO_SDO_AB_CMD`), and CANopenNode's client doesn't fall back. Now segmented only: `CO_CONFIG_SDO_CLI=0x0B`
   (no client block code), `CO_CONFIG_FIFO=0x01` (no alt read / CRC, which only client block needs). Up to 1 KB, so
   at most ~150 round trips; an opt-in block flag can come later if something needs it. gtest
   `SdoBufferTransfersAreSegmented` (40-byte write and read between two local hubs, initiate 0x21 / 0x40 on the
   wire); nucleo: the 0x1008 read from a server without block transfer works.
   Also: `sdo_block_size` sets CANopenNode build flags, so it applies to every hub of the firmware. Validation now
   rejects different values on two hubs, and `enable_sdo_block()` takes a hub's configured value over a
   component's default (the mapper's 16), whatever the codegen order.
9. **Codegen hooks:** `enable_sdo_server()`, `enable_pdo(tpdo=N, rpdo=M)`, and a defined point where other
   components add OD entries before the hub's `setup()` (from codegen / constructors, or a setup priority above
   `PROCESSOR`). **Done** (2026-10-31, `canopen-next`, a comment on `CANopenComponent` in `canopen.h`, to fold into
   PR 8): no `enable_sdo_server()` (the server is always on, PR 7) and no PDO counts in codegen (`enable_pdo()`
   only compiles the PDO code; the records follow the `configure_*()` calls at runtime). The contract: build-time
   parts by the `enable_*()` codegen helpers; OD entries / handlers, `configure_tpdo/rpdo()`, `monitor_node()`,
   identity from codegen setters (run in `main()` before `App.setup()`; OTA, remote logs) or from a `setup()` with
   priority above `PROCESSOR` (the mapper: `PROCESSOR + 1`); later calls log an error and are ignored (already
   in the code, `CanopenOD.AfterFinalize` tests the OD side).

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
| 10 | 0x1010/0x1011 store/restore (PDO parameters, preferences); done, `canopen-next` | ~105 (tests ~110) |
| 11 | loopback between local nodes; done, `canopen-next` | ~55 (tests ~180) |
| 12 | SDO server block transfer, `sdo_block_size`; done, `canopen-next` (the size cap needs the fork's patch upstream) | ~40 (tests ~150) |

PRs 7-9 are the minimum for the mapper against can2mqtt; 10-12 for full parity.

All of them are developed on one branch, `canopen-next` in `~/repos/esphome` (from `canopen-4-profile`, one commit per
PR), and cut into stacked PR branches when they go out. `test/stm32l4_canopen.yaml` builds `canopen` from
`../../esphome`, i.e. whatever branch is checked out there.

User docs: `~/repos/esphome-docs` branch `canopen-full` (renamed from `canopen-next` 2026-10-31, pushed to
mrk-its/esphome-docs, no PR; from `canopen`, the PRs 1-3 page) adds to `components/canopen.mdx` what PRs 7-9 add for YAML users: identity and device
information, the SDO server, an object dictionary table, `nmt_startup`, `on_nmt_state_change`,
`get_nmt_state()` / `set_nmt_state()`. The C++ APIs for other components (PR 4 profile API, PR 8 OD handlers,
PR 9 PDO configuration) aren't documented there. Split per PR like the code when the PRs go out. Gaps filled
(2026-10-31): several nodes on one bus (loopback), the PDO and 0x1010 / 0x1011 rows, a "PDOs" section
(SDO changes, save / load, storage), `sdo_block_size` one value per firmware, the intro list (raw frames, objects and
PDOs of other components, no EMCY producer / SYNC / LSS / EDS), a CANopen link on the CAN Bus page.

Entity commands (0x2002+16N, 2026-10-31): one write-only, RPDO-mappable sub per state with the same size, storage in
the entity, run by the entity's `od_write()` when the write is complete (SDO, RPDO, `write_od()`). Semantics as the
old component: sensor / number / binary sensor publish the value (a number is *published*, not `make_call()`, so its
`set_action` doesn't run; to fix, see the open points), switch on / off, light state / brightness / color temperature, cover stop / open / close /
position / tilt, alarm disarm / arm / trigger (127); a scaled NaN (255 / 65535) is ignored. Tested on the nucleo with
local SDO writes (`test/stm32l4_canopen.yaml`). CANopenNode quirk: the *local* SDO client reads a write-only entry
anyway (`CO_SDOclientUpload()` sets the write-only abort but then reads and overwrites the result); the SDO server
checks the access correctly.

Mapper tests: `test/test_canopen_entities.py` (in `pytest test/`). It runs codegen and validation through the upstream
venv's CLI (`esphome compile --only-generate`, `esphome config`) with the hub from `~/repos/esphome` as a local external
component (`$ESPHOME_UPSTREAM_VENV`, `$ESPHOME_CANOPEN_HUB`; skipped without them). It checks: one entity of every
domain to the `add_*()` calls in index order; the 8 `configure_tpdo()` calls (COB-IDs, mappings, the empty ones
disabled); the layout hash; the identity after the hub's default; the version string; the entity count define; and
the validation errors (duplicate index, unsupported domain, TPDO over 8 bytes, caps, size, ranges). It also checks
that `layout.py` is identical to the old component's copy. ESP8266 build check: `test/esp8266_mapper.yaml`.

Open points for `canopen_entities` itself (not in the steps yet):
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
  `canopen: entities` and hub `canopen:` + `canopen_entities: entities`, plus the target's OD writer flag),
  `rpdo: {node, entity, state, cmd}` on entities, remote state slots 0x4000+n (`RemoteSlot`, an `OdObject`, in a
  fixed `std::array` so listeners can attach before the slot sizes are set), `binary_sensor` / `sensor` /
  `text_sensor` **`platform: canopen_entities`** (was `platform: canopen`: a platform module must live in the
  component's directory, and `canopen` is now the upstream hub), and the `canopen.<domain>.<action>` actions (names
  kept at first; `canopen_entities.<domain>.<action>` since 2026-11-01, off the hub's `canopen.` prefix; `RemoteCommandAction` with at most 3 writes, sent with `send_od_write()`; an action to a node with the OD
  writer off is rejected). RPDOs through the hub's `configure_rpdo()`, one per remote (node, TPDO), mapping the
  remote TPDO's **full length** (CANopenNode reports a longer frame with an EMCY): gaps and the tail are single
  dummy entries (index 0x0005, sub 0, any length); the plan (`remote.rpdo_plan()`, ESPHome-free) rejects overlaps
  (a state in a slot and in an `rpdo:`), more than 8 entries. No 4-RPDO limit any more (hub: 32). Tests:
  `test/test_mapper_remote.py`, codegen / errors in `test/test_canopen_entities.py`; nucleo against the real dimmer4
  (node 0x37, old component): slots follow light5, `canopen.light.turn_off` turns it off, `test_switch` follows
  light4 by `rpdo:`, "Biuro toggle" button in HA. Not ported: `canopen.entity_cmd`, `template_entities` (open points
  below).
- **Remote logs ported** (2026-10-31): `canopen_entities: remote_logs:` (was `canopen: remote_logs:`; same
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
- **OTA ported** (2026-10-31): `ota: - platform: canopen_entities` (`canopen_id`), OD 0x3000 as before (can2mqtt
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
  speed with bigger blocks / a bigger server buffer. Planned: move to the hub on CiA 302-3 objects,
  [canopen_ota.md](canopen_ota.md).
- **MPDO for the OD writer (changes the wire protocol):** the old format's `0x500 + target` is also the target's
  TPDO 7 (`0x500 + node`), so a TPDO 7 frame whose first byte equals another node's id is applied there as an OD write,
  and the ID has several producers. So: the CiA 301 MPDO format (destination address mode: byte 0 = `0x80 | target`;
  index LE; subindex; 4 data bytes) on the **sender's** TPDO 7 (`0x500 + sender`). Receiving: both formats (2026-11-03).
  Sending: `od_writer_mpdo: true`, default false (2026-11-03), rejected next to an entity on TPDO 7; targets in the
  old component's YAML format keep getting old frames (`RemoteCommandAction::set_old_format()`), and the YAML can't
  tell whether a migrated node's firmware is new enough, so update the targets first. Not on hardware yet. Later:
  default true, then drop the old format.
- **Raw RPDO linking ported** (2026-10-31): `rpdo: {node_id, tpdo, offset, cmd, tpdo_size}` on mapper entities, one
  byte at `offset` of the remote TPDO into command `cmd`, a source of the same `remote.rpdo_plan()` as the named form
  (grouped with named entries and slots of the same node / TPDO, overlaps rejected). New: optional `tpdo_size` (1..8),
  the remote TPDO's length, which the RPDO then maps in full; unset, the RPDO ends at the last mapped byte and a longer
  frame is still processed, but CANopenNode sets `CO_EM_RPDO_WRONG_LENGTH` (one EMCY). A named entry of the same TPDO
  gives the size too; different sizes are an error. Stricter than the old component: the command must be 1 byte
  (was any size), `node_id` 1..127, `tpdo` 0..7, `offset` 0..7. Document it with the user migration
  (`configuration.md`, "RPDO schema"). Tests: `test_rpdo_plan_raw*` in `test/test_mapper_remote.py`,
  `test_codegen_raw_rpdo` / `test_invalid_raw_rpdo` in `test/test_canopen_entities.py`. Not on hardware yet.
- `template_entities` (custom OD entries with typed command triggers, states, metadata);
- `canopen.entity_cmd` (a command to another node's entity, on `write_sdo` or the OD writer frame);
- `csdo:` and its lambda API (`setup_csdo`, `csdo_recv`, `csdo_send_*`): map onto the hub's SDO client; custom
  tx/rx IDs need the pinned channels (device_api_canopennode.md, "Open points");
- user migration: done (`docs/migration.md`, 2026-11-01);
- until the hub is merged, where the mapper gets the hub from: done, a synced copy in this repo (`components/canopen`,
  2026-11-01).

## Size comparison with the old component (2026-10-31)
Compile-only, equivalent configs: old `components/canopen` vs the hub (`canopen-next`) + `canopen_entities`, and a
baseline without CANopen **and without OTA** (so both deltas include the platform's OTA backend).
- ESP8266 (d1_mini + mcp2515) and ESP32-C3 (esp-idf), upstream venv: the `test/esp8266_mapper.yaml` content (3
  entities, `remote_nodes`, one `rpdo` link, `remote_logs: {}`, two remote actions, a `platform: canopen` /
  `canopen_entities` text sensor, OTA); old with `sdo_block_transfer_size: 16` (= the mapper's default).
- STM32L4 (Zephyr, fork venv): `test/stm32l4_biuro_switch.yaml` (no entities, dimmer4 remote, `remote_logs` DEBUG /
  block 16, block size 60, MCUboot OTA, `safe_mode`); old with `entities: []`, `sdo_block_transfer_size: 60`.

| Target | Baseline flash / RAM | Old: Δflash / ΔRAM | New: Δflash / ΔRAM (static) | Flash new - old |
|---|---|---|---|---|
| ESP8266 | 273.9 KB / 28.5 KB | +49.1 KB / +15.3 KB | +50.3 KB / +4.8 KB | +1.2 KB |
| ESP32-C3 | 213.7 KB / 52.0 KB | +53.1 KB / +13.8 KB | +51.7 KB / +3.4 KB | -1.3 KB |
| STM32L4 (app image) | 82.3 KB / 18.7 KB | +35.3 KB / +5.5 KB | +38.3 KB / +2.9 KB | +3.0 KB |

Flash is at parity. The static RAM numbers are not comparable:
- 8 KB of the old ESP RAM is the OTA inflate buffer (`CanopenOTAComponent`, static); the mapper allocates it when an
  OTA starts (idle -8 KB, same peak). Compression is off on STM32, so not there.
- CANopenNode allocates in `CO_new()` (heap, not in the build report). Measured with a 32-bit host build of
  `CANopen.c` with the STM32 build's `-DCO_CONFIG_*` flags (1 RPDO, 8 TPDOs, no HB consumer): **4948 bytes** (5140
  with the SDO client). Of that, a PDO object is 380 bytes (9 = ~3.4 KB) and the SDO server 1016 bytes (the
  900-byte block buffer).
- Not measured (needs free heap on a running device): the hub's runtime OD / PDO vectors, the old component's vectors
  and `new` entities.

Like for like at idle (without the inflate buffer, with `CO_new()`): ESP8266 ~7.3 KB old vs ~10 KB new, STM32L4
~5.5 KB vs ~7.8 KB, i.e. the new one is ~2.5 KB larger plus the OD vectors. Cheapest saving: the mapper configures all
8 TPDOs even when empty (all of them in the biuro config); trailing unused TPDOs could be left out
(`CO_CANopenInitPDO()` only needs contiguous records from the first), ~380 bytes each.

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
- PDO parameter storage on ESP8266 (hub PR 5, `pdo_storage.cpp`): the stored PDO parameters go to the small flash
  preference area (64 words, 128 with `esp8266: restore_from_flash: true`), 11 words per PDO (`PdoParams`, 44 bytes)
  plus a checksum word per RX/TX preference and the 2-word marker: about 5 PDOs fit without `restore_from_flash`, 11
  with it, minus other flash preferences (API key, `globals`, ...). If they don't fit, `make_preference()` returns an
  empty object and `save` (0x1010) fails at runtime (log "Storing the PDO parameters failed", SDO abort `ODR_HW`);
  the build doesn't warn, since the hub doesn't call `esp8266.const.reserve_preferences()` (the PDO count comes from
  runtime `configure_tpdo/rpdo()`). Fix: counts in codegen, e.g. `enable_pdo(rpdos, tpdos)` reserving the three
  preferences on ESP8266, or a hub helper the components call. Left out of the user docs for now (ESP8266 isn't a
  priority, 2026-10-31).

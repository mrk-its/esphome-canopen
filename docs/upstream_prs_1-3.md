# Upstream PRs 1-3: the CANopenNode-based `canopen` hub

What is built, tested and ready for review in the first three upstream ESPHome PRs: a CiA 301 node on CANopenNode
with heartbeat monitoring, raw frames and events, an SDO client and an NMT master. This file describes the result;
the plans and their history are in [canopennode.md](canopennode.md) (overall port, PR chain),
[minimal_canopennode.md](minimal_canopennode.md) (step 1) and [device_api_canopennode.md](device_api_canopennode.md)
(step 2). The PRs after these (profile API, `cia402`, PDO) are in [cia402.md](cia402.md).

## Branches
All in `~/repos/esphome`, branched from `upstream/dev`, one commit each, stacked, pushed to `origin`
(mrk-its/esphome). No PRs opened yet.

| PR | Branch | Commit | Content | Size (component / tests) |
|---|---|---|---|---|
| 1 | `canopen-1-node` | `2d145d6` | CANopenNode integration, NMT, heartbeat producer, `heartbeat_clients`, `on_node_event` | 845 / 175 |
| 2 | `canopen-2-frames` | `7c18230` | `on_heartbeat`, `on_emcy`, `add_frame_listener()`, `send_frame()` | +136 / +109 |
| 3 | `canopen-3-sdo` | `30e380a` | SDO client, NMT master, `canopen.sdo_write`, `canopen.nmt`, `add_record()` | +631 / +107 |

- Source of truth is `components_next/canopen` in this repo (tested on hardware); each branch is cut from it by
  removing the later parts (synced with the branches on 2026-10-30). PR 4 (`canopen-4-profile`, the profile API)
  is described in [cia402.md](cia402.md), "PR 4".
- User docs: `~/repos/esphome-docs` branch `canopen` (`b632805`, pushed to mrk-its/esphome-docs, no PR), one page
  `src/content/docs/components/canopen.mdx` covering PRs 1-3 plus an index entry. To be opened together with PR 1.
- Gist for the #21533 author (secret): https://gist.github.com/mrk-its/8c295e2847c7586954facd1cbc5d5bbe (`cia402`
  proposal, links to the three branches and the docs page).

## Files

| File | PR | Content |
|---|---|---|
| `__init__.py` | 1-3 | schema, `to_code`, `add_canopennode_library()`, `enable_sdo_client()` / `enable_nmt_master()`, actions |
| `canopen.h` / `canopen.cpp` | 1-3 | `CANopenComponent`: setup, OD, processing, monitoring, frames, NMT master |
| `sdo_client.cpp` | 3 | request queue and the `CO_SDOclient` state machine |
| `automation.h` | 3 | `SDOWriteAction`, `NMTAction` |
| `types.h` | 1-3 | public types, no CANopenNode headers: `NMTState`, `NodeEvent`, `NMTCommand`, `SDOResult`, `SDOReply`, `Emergency`, callbacks, decoders |
| `od.h` / `od.cpp` | 1, 3 | runtime OD builder: `add_var`, `add_array`; `add_record` in PR 3 |
| `driver_can.cpp` | 1 | CANopenNode driver functions on ESPHome `canbus` |
| `canopennode_port/CO_driver_target.h`, `library.json` | 1 | the CANopenNode target header (C, `#pragma once`) |

## CANopenNode integration
- **Library:** `cg.add_library("CANopenNode", None, CANOPENNODE_SOURCE)`, the fork
  `https://github.com/mrk-its/CANopenNode.git` pinned to `f38ec8a` (branch `esphome` = upstream master `9b8beed` +
  `library.json`, no stack changes). The fork exists only because ESPHome's Zephyr and native ESP-IDF library
  converters require a manifest; the `library.json` could go to CANopenNode upstream as is. `canopennode_path:` (an
  absolute path) uses a local checkout instead, for development.
- **Port header** in the component. Project `-I` flags don't reach converted libraries (only `-D`/`-W` do), so
  `add_canopennode_library()` adds its directory one way per backend:
  - ESP32 (ESP-IDF and Arduino): an IDF component `canopennode_port` generated in the build dir (header copy +
    `idf_component_register(INCLUDE_DIRS ".")`; esphome gitignores `CMakeLists.txt`), added with
    `add_idf_component()`. Converted libraries REQUIRE every project-managed component, and a local component is
    named after its directory;
  - Zephyr: a header-only library (`library.json`, `includeDir: "."`), Zephyr exports include dirs globally;
  - PlatformIO (ESP8266, RP2040, host): additionally `-I<dir>`.
- **Configuration** only through `CO_CONFIG_*` macros: the header sets the minimal defaults (each in `#ifndef`) and
  codegen adds build flags for the optional parts. Build flags are global, so CANopenNode and our `.cpp` files see the
  same values. Defaults: `CO_MULTIPLE_OD`; NMT with change callback; HB consumer with multi callbacks and query
  functions; EM producer; PDO, SYNC, TIME, storage, LEDs, LSS and the rest off. No SDO server in PRs 1-3.
- **Driver** (`driver_can.cpp`): `CANptr` is the component, no globals. `CO_CANsend` → `canbus::send_data(id, false,
  rtr, data, len)` (the `(data, len)` overload is in `dev`, not in 2026.9.x); a frame the platform refuses stays in its
  TX buffer (`bufferFull`) and `CO_CANmodule_process` resends it, a software TX queue on every platform. RX: the
  canbus callback (main loop) dispatches standard-ID frames to the matching CANopenNode RX buffers, then processes
  the stack right away (low SDO latency).
- **Runtime OD** (`CO_MULTIPLE_OD`): entries are collected and turned into a sorted `OD_entry_t[]` before
  `CO_CANopenInit`; nothing can be added afterwards. Entries: 0x1000, 0x1001, 0x1014 (EMCY COB-ID), 0x1016 (HB
  consumer, only with monitored nodes), 0x1017 (producer heartbeat), 0x1280 (SDO client, PR 3).
- **Init:** `CO_new` → `CO_CANinit` → `CO_CANopenInit(..., CO_NMT_STARTUP_TO_OPERATIONAL, ...)` →
  `CO_CANsetNormalMode`; errors logged with the failing OD index. `loop()` runs `CO_process` with the elapsed time;
  `CO_RESET_APP` → `App.safe_reboot()`, `CO_RESET_COMM` is only logged.

## PR 1: node and heartbeat monitoring
- YAML: `node_id` (1..127), `canbus_id`, `heartbeat_interval` (default 5 s, ≤ 65535 ms), `heartbeat_clients:
  [{node_id, timeout}]` (≤ 65535 ms, the own id is rejected), `on_node_event`, `canopennode_path`. `MULTI_CONF`:
  several nodes, e.g. on different buses.
- The node starts operational, follows NMT commands from a master and sends its heartbeat.
- **Monitoring** on CANopenNode's HB consumer: `monitor_node(node, timeout_ms)` before setup (codegen calls it for
  `heartbeat_clients`); duplicates are merged with the shortest timeout (CANopenNode rejects duplicates in 0x1016).
  Events from the consumer callbacks: started → `ONLINE`, timeout → `OFFLINE`, NMT changed → `NMT_CHANGED` (also right
  after `ONLINE` and `OFFLINE`), remote reset (boot-up) → `RESET`. `ONLINE`'s state is read from the consumer entry,
  since `CO_HBconsumer_getNmtState()` is valid only once the entry is active.
- API: `add_on_node_event_callback((node_id, NodeEvent, NMTState))` (the `on_node_event` trigger, via
  `LazyCallbackManager` / `build_callback_automations`), `get_node_state(node)`, `is_node_online(node)`;
  `nmt_state_to_string()`, `node_event_to_string()`.
- Python exports for later components: `CANopenComponent`, `CONF_CANOPEN_ID`, `CONF_NODE_ID`.
- Also in the PR: `script/ci-custom.py` exempts `CO_driver_target.h` from the namespace and `#define` lints (a C
  header), and `CODEOWNERS`.

## PR 2: raw events and frames
- `on_heartbeat` (`node_id`, `state`) for every heartbeat frame and `on_emcy` (`node_id`, `const Emergency &`:
  `error_code`, `error_register`, `manufacturer_data[5]`) for every EMCY frame (0x81..0xFF), from any node, monitored or
  not. Decoded by `decode_heartbeat()` / `decode_emcy()` in `types.h`. A node doesn't receive its own frames.
- `add_frame_listener(can_id, mask, FrameCallback, context)`, before or after setup: every standard-ID data frame with
  `((id ^ can_id) & mask) == 0`, after CANopenNode has seen it. Logs a warning if the filter overlaps an ID that
  CANopenNode receives itself.
- `send_frame(can_id, data, len)`: sends now; false if the platform refuses it. Not queued or retried (unlike the
  stack's frames); a TX ring is designed for later ("Later: TX ring" in device_api_canopennode.md).
- `tests/components/canopen/__init__.py` makes the C++ unit tests build: the runner suppresses `to_code` and gives
  `MULTI_CONF` components an empty list, so the override sets `multi_conf = False` and its `to_code` calls only
  `add_canopennode_library()`.

## PR 3: SDO client and NMT master
- **Optional parts:** compiled only when used. `enable_sdo_client()` (define `USE_CANOPEN_SDO_CLIENT`,
  `-DCO_CONFIG_SDO_CLI=0x0F` = enable/segmented/block/local, `-DCO_CONFIG_FIFO=0x07`, `-DCO_CONFIG_CRC16=0x01`) and
  `enable_nmt_master()` (`USE_CANOPEN_NMT_MASTER`, `-DCO_CONFIG_NMT=0x03`), each once per build; today called by the
  actions, later by profile components. The C++ API of a part exists only under its `#ifdef`, so a component that
  forgets to enable it fails to compile.
- **SDO client:** one channel (`CNT_SDO_CLI = 1`, OD 0x1280 RECORD via `add_record()`, set up per request with
  `0x600/0x580 + node`), a 16-entry FIFO queue, `response_timeout` per transfer (default 500 ms).
  - `read_sdo(node, index, sub, expected_size, cb, ctx)` / `read_sdo<T>()`: values of 1, 2 or 4 bytes; another width
    → `SIZE_MISMATCH` (0 = any ≤ 4). `SDOReply::as<T>()` sign-extends signed types, bit-casts `float`.
  - `write_sdo(node, index, sub, value, size, cb, ctx)` / `write_sdo<T>()`.
  - `read_sdo_data()` (up to 1024 bytes) / `write_sdo_data()` (data copied): segmented, block above 32 bytes.
  - **Exactly one callback per request.** Rejections (queue full, bad arguments, stack not running) call back
    synchronously before the call returns false; accepted requests call back from processing after their slot is
    freed, so the callback can queue the next request.
  - `cancel_sdo(ctx)`: drops queued requests of `ctx` and aborts its transfer in flight, no callbacks afterwards. If
    CANopenNode couldn't send the abort frame (its TX buffer busy), `abort_sdo_()` sends `0x80 ... 0x08000000` itself,
    and the next request to that node waits 50 ms.
  - Requests to the own node id use the local OD (`CO_CONFIG_SDO_CLI_LOCAL`), no frames.
  - Results: `SUCCESS`, `ABORT` (with `abort_code`), `TIMEOUT`, `SIZE_MISMATCH`, `TRANSPORT_ERROR`, `REJECTED`;
    failures logged at WARN.
- **NMT master:** `send_nmt(node, NMTCommand)` (`START`, `STOP`, `PRE_OPERATIONAL`, `RESET_NODE`,
  `RESET_COMMUNICATION`; node 0 = all, the own id or 0 also apply to this node).
- **Actions:**
  - `canopen.sdo_write`: `node_id`, `index`, `subindex` (0), `value`, all templatable; `size` 1/2/4 (default 4);
    `on_success`, `on_error` (`result`, `abort_code`). A constant must fit in `size` bytes; negative constants are
    written as two's complement. Non-blocking: the action continues, the triggers run when the transfer ends.
  - `canopen.nmt`: `node_id` (0..127, templatable), `command` (`start`, `stop`, `pre_operational`, `reset_node`,
    `reset_communication`).
- `od.h`: `add_record(index, {{sub, attr, data, len}, ...})` → `ODT_REC` (`ODA_MB` for ≥ 2 bytes).

## Tests
- **Upstream checks** (venv `~/repos/esphome-upstream-venv`, `PATH=…/bin:$PATH PYTHONPATH=~/repos/esphome`, all pass on
  each branch): `script/ci-custom.py`, ruff, clang-format 23.1.3, pre-commit hooks;
  `script/test_build_components.py -c canopen --isolate canopen` builds esp32-idf, esp32-ard, esp8266-ard and
  rp2040-ard (both with mcp2515) without warnings, from `tests/components/canopen/common.yaml` (all options, triggers,
  lambdas and both actions).
- **Python** `tests/component_tests/canopen/test_canopen.py` (6 tests in PR 1, 8 in PR 3): generated code for the
  options, triggers and actions (`cg.templatable` wraps constants in lambdas), validation (own id in
  `heartbeat_clients`, ranges, value doesn't fit `size`, negative masking).
- **C++ unit tests** (`script/cpp_unit_test.py canopen`, gtest): `frames_test.cpp` (heartbeat/EMCY decoders, PR 2),
  `sdo_reply_test.cpp` (`SDOReply::as<T>()`, PR 3).
- **This repo** (`components_next`): `test/test_od.py` (OD builder against CANopenNode's `CO_ODinterface.c`) and
  `test/test_frames.py`; build checks `test/esp32_canopen.yaml` (+ `-s framework arduino`) and
  `test/esp8266_canopen.yaml` with upstream `dev`.
- **Hardware** (`test/stm32l4_canopen.yaml`, nucleo_l476rg, `zephyr_can` 125 kbit/s, node 0x7f, esphomeplatformzephyr
  venv), on a bus with ~11 nodes on the old canopen-stack component:
  - PR 1: CANopenInit OK, operational, heartbeats out; node 0x01 `ONLINE` / `NMT_CHANGED`; bus error-active with
    0 TX errors.
  - PR 2: heartbeats of all nodes reported; a TPDO1 listener caught node 0x32's `0x1b2`; the overlap warning for
    0x701; `send_frame()` every 5 s with 0 TX errors.
  - PR 3: `test/sdo_test.h` against node 0x61 and the own node, **24/24 pass**: typed and buffer reads, size
    mismatch, abort, timeout (absent node 0x7E), write of 0x61's 0x1017 checked in its measured heartbeat period and
    restored, local transfers, queue limit with one callback per request, cancel queued and in flight (with a read
    right after), NMT pre-operational / start checked in the heartbeats, both actions (script after 75 s).
- **Not covered:** `OFFLINE` / `RESET` events on hardware in the automatic test, block transfers over 32 bytes,
  `RESET_NODE` in the automatic test (tested once by hand), the fallback abort frame of `cancel_sdo()` (the final run
  took the normal path), ESP32 / ESP8266 hardware, builds with the optional parts off checked in
  `compile_commands.json`.

## Lessons from the hardware tests
- **canopen-stack 0x1017 bug** (old nodes): 0x1017 is a direct value and a value of 0 reports size 0, so after writing
  0 every 0x1017 access aborts with 0x08000020 until a reboot (an NMT reset doesn't help). Never write 0 there; the
  test measures the heartbeat period from the frames and writes 0x1017 only from that measured value, and doesn't send
  `RESET_NODE`.
- **Cancel in flight:** CANopenNode sends the abort only if its TX buffer is free; otherwise the server stays in the
  transfer and rejects the next request (0x05040001). Fixed by the direct abort frame and the 50 ms guard.

## Decisions
- No CANopenNode changes beyond packaging; configuration through build flags.
- Small PRs: the chain above instead of one ~2000-line PR; `send_frame()` without a queue, one SDO channel, no
  priorities, no SDO server yet.
- `on_node_event` replaces the old `on_hb_consumer_event`; the remote states (`platform: canopen`, `0x4000+n`
  slots, `canopen.<domain>.<action>`) aren't in the hub, they move to the entity mapper (`esphome_entity_mapper`).
- Several `canopen:` nodes per build are allowed (unlike #21522's one hub per bus).

## Open
- Coordination with #21522 / #21533 (another author's `canopen` hub and `cia402` on it); the drafted comment for
  #21533 with the gist link isn't posted.
- When to open the PRs; the docs PR collides with esphome.io #7889 (same `canopen.mdx`).
- Keep `canopennode_path` in the upstream version?
- Lambda-only use of the SDO client / NMT master: today they're compiled only if an action uses them. Proposal: hub
  options `sdo_client: true` / `nmt_master: true` (or wait for `register_node()` in PR 4).
- Astro build of the docs page not run (the repo pins Node 24); `script/lint.mjs` passes.

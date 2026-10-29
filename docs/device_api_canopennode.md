# CANopenNode step 2: device API (SDO client, NMT master, node monitoring, raw frames)

## Context
Step 1 ([minimal_canopennode.md](minimal_canopennode.md)) gave `components_next/canopen`: NMT, heartbeat
producer/consumer, the driver, a runtime OD with VAR/ARRAY entries.

Goal of this step: the API profile components need to drive other CANopen devices, CiA 402 drives first. The
reference is esphome/esphome#21522 (hub with SDO client, NMT, heartbeat/EMCY events, `CANopenNode` base class) and
esphome/esphome#21533 (`cia402` on top of it). Both are fresh and unmerged, so **we keep their functionality, not
their API**. The API below fixes what makes `cia402` complicated or slow there:
- it polls statusword/mode/position/velocity over SDO round-robin and re-reads the statusword every 50 ms while it
  waits for state bits → PDO support (raw frame listeners + send);
- every profile keeps its own `last_heartbeat_ms_` and checks timeouts only while a sequence runs, and a drive
  reboot (boot-up, lost configuration) isn't an event → node monitoring in the hub on CANopenNode's HB consumer;
- `halt()`/`quick_stop()` wait behind the request in flight (up to the response timeout) and behind a shared queue
  → controlword by PDO (`send_frame()`), or `cancel_sdo()` + write for drives without that RPDO;
- callers check the response width and sign-extend themselves → expected size / typed reads;
- a rejected request returns false without a callback, so every caller has two result paths, and stopped sequences
  need `pending_stop_`/`pending_sequence_` bookkeeping for replies still in the queue → one result path,
  `cancel_sdo()`.

Decisions taken (with the user, 2026-10-29):
- Functionality of #21522/#21533 is the target; their API may change. `cia402` gets ported onto this API later
  (its own step, after this one).
- The new component drops the old remote states (`platform: canopen` entities reading other esphome-canopen nodes,
  `0x4000+n` slots, `canopen.<domain>.<action>`). No SDO-polled sensor for now.
- This step comes before the SDO server step ([sdo_server_canopennode.md](sdo_server_canopennode.md), now step 3);
  it's tested against node 0x01 on the bus, no host bridge needed.

Out of scope: SDO server (`CNT_SDO_SRV` stays 0; step 3), our own PDOs (entity mapper, step 7), the host OD writer
and local loopback (step 8, but the API here is shaped for them), `cia402` itself and its remote PDO config helpers
(`configure_rpdo`/`configure_tpdo`), several or pinned SDO client channels, urgent SDO requests (see "Open points").

## Constraints from the later steps
The API must fit these without changes:
- **SDO server (step 3):** CANopenNode's server and our client run side by side (separate RX/TX buffers). Client
  requests to our own node id never go to the bus: with `CO_CONFIG_SDO_CLI_LOCAL` (set by codegen, no fork change) they read/write our OD
  through the OD IO, so the step 4 `OdHandler` extensions fire exactly as for a remote SDO write.
- **Entity mapper (steps 7/8):**
  - our TPDOs are CANopenNode `CO_TPDO` objects (OD-backed); the raw `send_frame()` here is for frames the OD
    doesn't describe (another device's RPDO, e.g. a drive's controlword). CANopenNode's frames are retried by
    `CO_CANmodule_process`; `send_frame()` isn't (see "Raw frames", TX ring for later);
  - the host OD writer (COB-ID `0x500 + node`) is a frame listener of this step's kind (`add_frame_listener`), not
    special code in `on_frame_`;
  - **several `canopen:` nodes on one bus** keep working (the old component supports it): no "one hub per bus"
    rule as in #21522; instead `node_id` must be unique per `canbus_id`. Step 8 adds the loopback in the driver
    (`CO_CANsend` and `send_frame()` deliver to the other local instances), which also makes SDO, heartbeat and
    raw listeners work between local nodes without extra code.
- **Re-init on `CO_RESET_COMM` (later):** registrations (listeners, monitored nodes, callbacks) live in our
  component, not in CANopenNode objects, and are re-applied after `CO_delete` + `CO_new`.
- Only the node core (`canopen.cpp`, `od.cpp`, `driver_can.cpp`, `sdo_client.cpp`) includes CANopenNode headers;
  `canopen_node.h` and profile components see only the types below.

## Optional parts: compiled only when used
Like the old component (canopen-stack built with `-DUSE_CSDO=0`), a node that only exposes its own entities
shouldn't pay for client-side code. CANopenNode guards each service with its `CO_CONFIG_*` macro (e.g. all of
`301/CO_SDOclient.c` is inside `CO_CONFIG_SDO_CLI_ENABLE`, and `CANopen.c` checks the same macros), and the port
defines them in `#ifndef`, so codegen turns them off with build flags. Build flags are global, so the library and
our `.cpp` files see the same values (checked in step 1); with several `canopen:` blocks in one build a part is on if
any of them needs it.

| Part | Needed when | Off: build flags / defines |
|---|---|---|
| SDO client | a `canopen.sdo_write` action or a profile component asking for it | `-DCO_CONFIG_SDO_CLI=0`, no `USE_CANOPEN_SDO_CLIENT`, `CNT_SDO_CLI = 0`, no 0x1280 (on: the flag with `CO_CONFIG_SDO_CLI_LOCAL`, see below) |
| HB consumer | `heartbeat_clients` or any monitored node | `-DCO_CONFIG_HB_CONS=0`, `CNT_HB_CONS = 0`, no 0x1016 |
| NMT master | `canopen.nmt` or a profile component | `CO_CONFIG_NMT` without `CO_CONFIG_NMT_MASTER` |
| SDO server (step 3) | always (can2mqtt) | - |
| frame listeners, raw heartbeat/EMCY callbacks | always compiled | a few bytes when unused (empty vector / callback managers) |

- On: `cg.add_define("USE_CANOPEN_SDO_CLIENT")` / `USE_CANOPEN_HB_CONSUMER` / `USE_CANOPEN_NMT_MASTER`. The API
  that needs a part (`read_sdo`, `write_sdo`, `cancel_sdo`, `monitor_node`, node events, `send_nmt`) is declared
  only inside its `#ifdef`, in `canopen.h` and `canopen_node.h`: a profile that forgets to ask fails to compile
  instead of failing at runtime.
- Profile components ask through one Python helper, `await canopen.register_node(var, config, sdo=True,
  nmt=True, heartbeat_timeout=...)`, called from their `to_code`: it sets the defines and calls
  `monitor_node()`.
- The SDO client has exactly one channel (`CNT_SDO_CLI = 1`), no YAML option. That's enough for CiA 402: a drive's
  SDO server handles one transfer at a time anyway, the per-drive configuration sequences are short, and the
  time-critical traffic (controlword, statusword, position) goes by PDO. More channels only help with several
  drives polled over SDO (drives whose PDOs can't be remapped); they come later, in one PR with pinned channels
  (see "Open points"). Upstream prefers several small PRs, so this step stays minimal.
- Each of the three needs a build with the part off (the CANopenNode sources must compile with the macro at 0); I
  expect it, since upstream supports those configurations, but it isn't checked yet.

## Changes

### No CANopenNode fork changes
- Local SDO client transfers use the upstream option `CO_CONFIG_SDO_CLI_LOCAL`, set from codegen: when the SDO
  client is on, `to_code()` adds `-DCO_CONFIG_SDO_CLI=(CO_CONFIG_SDO_CLI_ENABLE|CO_CONFIG_SDO_CLI_SEGMENTED|
  CO_CONFIG_SDO_CLI_BLOCK|CO_CONFIG_SDO_CLI_LOCAL)` (the port's default plus `_LOCAL`; the port defines it in
  `#ifndef`). Rule for the port: change the fork only when a build flag can't do it.
- Why not over the bus: the controller doesn't receive its own frames (no answer → `TIMEOUT`), and from step 3 on
  can2mqtt uses the same `0x600 + node_id` server channel (two transmitters of one CAN ID, two interleaved
  transfers in our one-at-a-time server).

### `od.h` / `od.cpp`
- (done, moved to PR 1) `add_record(index, subs)`: `RecordSub {uint8_t sub; OD_attr_t attr; void *data; OD_size_t len;}` → `ODT_REC` + an
  array of `OD_obj_record_t` (sub0 included, `subEntriesCount` = number of subs, `ODA_MB` for len ≥ 2), stored in a
  `std::deque<std::vector<OD_obj_record_t>>`. Used for 0x1280 here, 0x1018/0x1200 in step 3, PDO records in step 7.

### Names
- (done, moved to PR 1) C++ `CanopenComponent` → `CANopenComponent` (the ESPHome spelling, as in #21522); Python exports
  `CANopenComponent`, `CANopenNode`, `CONF_CANOPEN_ID = "canopen_id"`, `CONF_NODE_ID` for profile components.

### `types.h`: public types (no CANopenNode headers)
- `enum class NMTState : uint8_t {BOOTUP = 0, STOPPED = 4, OPERATIONAL = 5, PRE_OPERATIONAL = 127, UNKNOWN = 255}`
  (the heartbeat byte values; also used for our own state).
- `enum class NMTCommand : uint8_t {START = 1, STOP = 2, PRE_OPERATIONAL = 128, RESET_NODE = 129,
  RESET_COMMUNICATION = 130}`.
- `enum class SDOResult : uint8_t {SUCCESS, ABORT, TIMEOUT, SIZE_MISMATCH, TRANSPORT_ERROR, REJECTED}`.
- `struct SDOReply {SDOResult result; uint32_t abort_code; uint32_t value; uint8_t size; const uint8_t *data;
  size_t len;}`: `value`/`size` for reads of ≤ 4 bytes (little-endian, zero-extended), `data`/`len` for buffer reads
  (valid only during the callback), `abort_code` only with `ABORT`/`TIMEOUT`.
- `using SDOCallback = void (*)(void *context, const SDOReply &)`. A function pointer + context like #21522: no
  `std::function` per request, so the queue stays a fixed array.
- `struct Emergency {uint16_t error_code; uint8_t error_register; std::array<uint8_t, 5> manufacturer_data;}`.
- `enum class NodeEvent : uint8_t {ONLINE, OFFLINE, NMT_CHANGED, RESET}` (RESET = a boot-up message from a node we
  monitor: it lost its runtime configuration).
- `using FrameCallback = void (*)(void *context, uint32_t can_id, const uint8_t *data, uint8_t len)`.

### `CANopenComponent`: public API
SDO client:
- `bool read_sdo(node, index, sub, uint8_t expected_size, SDOCallback cb, void *ctx)`:
  `expected_size` 1/2/4 → any other width gives `SIZE_MISMATCH`; 0 → any width ≤ 4.
- `template<typename T> bool read_sdo(node, index, sub, cb, ctx)`: `expected_size = sizeof(T)`; `SDOReply`
  gets `template<typename T> T as() const` (sign extension for signed `T`, `float` bit cast).
- `bool write_sdo(node, index, sub, uint32_t value, uint8_t size, SDOCallback cb = nullptr, void *ctx = nullptr)`
  and `template<typename T> bool write_sdo(node, index, sub, T value, ...)`.
- `bool read_sdo_data(node, index, sub, SDOCallback cb, void *ctx)` (segmented/block, result in `data`/`len`, up to
  the channel buffer) and `bool write_sdo_data(node, index, sub, const uint8_t *data, size_t len, cb, ctx)` (data
  copied into the request; block or segmented, see the implementation below). For strings, domains, OTA of other
  nodes.
- **One result path:** every call ends in exactly one callback, also a rejection (queue full, bad node/size, stack
  not initialised). A rejection has no queue slot: its callback runs synchronously, inside `read_sdo()`/`write_sdo()`,
  before the call returns false. Callers must allow that re-entry (it's the path `cia402` already takes by hand).
  Accepted requests call back from `process_()`, after their slot is freed.
- `void cancel_sdo(void *ctx)`: drops every queued request with that context **without** a callback; a transfer of
  that context in flight is aborted (`CO_SDOclient*` with `abort = true`, abort code `0x08000000`) and doesn't call
  back either. After `cancel_sdo()` returns, nothing calls back for `ctx`.
- No priorities: requests run in FIFO order. A stop command (controlword write) to a drive without an RPDO for it is
  `cancel_sdo(this)` + `write_sdo(...)`: it waits at most for another component's transfer in flight
  (`response_timeout`). Urgent requests that preempt other components: see "Open points".

NMT:
- `bool send_nmt(node, NMTCommand)` → `CO_NMT_sendCommand()` (node 0 = all; for our own id or 0 CANopenNode also
  applies it to us). `NMTState get_nmt_state() const` for our own state.

Node monitoring (CANopenNode HB consumer; done, moved to PR 1):
- `void monitor_node(uint8_t node, uint16_t timeout_ms)`: called from codegen, before `setup()`. All monitored nodes
  plus the YAML `heartbeat_clients` become the 0x1016 entries; duplicates are merged (CANopenNode rejects duplicate
  node ids in 0x1016), keeping the shortest timeout.
- One `LazyCallbackManager<void(uint8_t node, NodeEvent, NMTState)>`, fed from the HB consumer callbacks
  (`HeartbeatStarted` → ONLINE, `Timeout` → OFFLINE, `NmtChanged` → NMT_CHANGED, `RemoteReset` → RESET):
  `add_on_node_event_callback(F)` gets all monitored nodes (the YAML trigger), `add_on_node_event(node, F)` with
  `(NodeEvent, NMTState)` is a filter on top for profiles. Events are passed as CANopenNode reports them:
  NMT_CHANGED also comes right after ONLINE (unknown → state) and after OFFLINE (→ unknown). ONLINE's state is read
  from the consumer entry directly: `CO_HBconsumer_getNmtState()` returns it only once HBstate is ACTIVE, which is
  set after the started callback.
- `NMTState get_node_state(node)`, `bool is_node_online(node)` via `CO_HBconsumer_getIdxByNodeId` /
  `CO_HBconsumer_getState` / `CO_HBconsumer_getNmtState`.
- YAML: `heartbeat_clients` (→ `monitor_node()`; timeout max 65535 ms, uint16 in 0x1016; our own node id is
  rejected) and **`on_node_event`** (`node_id`, `event`, `state`), which **replaces `on_hb_consumer_event`**
  (decision 2026-10-29: one trigger with all events; the new component has no compatibility obligation, the old
  name goes away at the switch-over).

Raw events (any node, no registration; done, moved to PR 1):
- `add_on_heartbeat_callback(F)` with `(uint8_t node, NMTState)` for every heartbeat frame, and
  `add_on_emcy_callback(F)` with `(uint8_t node, const Emergency &)` for every EMCY frame (0x81..0xFF, 8 bytes),
  tapped in `on_frame_`. These are the YAML `on_heartbeat` / `on_emcy` triggers.

Raw frames (PDOs of other devices, later the host OD writer; done, moved to PR 1):
- `void add_frame_listener(uint32_t can_id, uint32_t mask, FrameCallback cb, void *ctx)`: registered before or after
  setup; `on_frame_` calls every matching listener (standard-ID data frames, not RTR) after the CANopenNode RX
  buffers (a frame can go to both). A small vector; logs a warning when it overlaps an ID CANopenNode receives
  itself (checked against its RX buffers at setup, or at registration after setup), since that's almost always a
  config mistake.
- `bool send_frame(uint32_t can_id, const uint8_t *data, uint8_t len)`: sends at once with `canbus::send_data()`;
  returns false if the platform refuses the frame (TX buffers full), and the caller decides whether to retry. No
  queue (decision 2026-10-29, keeps PR 1 small): `zephyr_can` (`tx_buffer_size`) and `esp32_can` (`tx_queue_len`)
  queue in software anyway; it matters on `mcp2515` (3 hardware buffers) or `zephyr_can` without the queue. No
  NMT-state check here; the profile decides (CiA 301: PDOs only in OPERATIONAL, see `get_nmt_state()`).
- **Later: TX ring for `send_frame()`.** If a user (`cia402` on mcp2515, the host OD writer) needs frames that
  aren't dropped when the controller is busy: a fixed ring in `driver_can.cpp` (e.g. 8 × `{id, len, data[8]}`).
  `send_frame()` sends at once only when nothing is pending (no CANopenNode `bufferFull` slot, empty ring), else
  appends (false only when the ring is full); `CO_CANmodule_process()` drains the CANopenNode slots first, then the
  ring, stopping at the first refusal. That also fixes the ordering gap of the simple version, where a raw frame
  can overtake a CANopenNode frame waiting for its retry.

### SDO client implementation (`sdo_client.cpp`)
- Channel: `CNT_SDO_CLI = 1`, `ENTRY_H1280` = 0x1280; OD 0x1280 RECORD:
  sub0 = 3, sub1/sub2 u32 COB-IDs (`0x80000000` = not valid), sub3 u8 server node id, `ODA_SDO_RW`.
- Queue: fixed `std::array<Request, 16>` (`Request {node, index, sub, kind (read/write/read_data/write_data),
  expected_size, value, std::vector<uint8_t> data (only write_data), callback, context}`); a FIFO ring.
- Dispatch in `process_()` (runs in `loop()` and right after every received frame): when the channel is idle, take
  the first queued request, `CO_SDOclient_setup(ch, 0x600 + node, 0x580 + node, node)`, then:
  - read: `CO_SDOclientUploadInitiate(ch, index, sub, timeout_ms, block)`, poll `CO_SDOclientUpload(...)`,
    `CO_SDOclientUploadBufRead` into the channel buffer;
  - write: `CO_SDOclientDownloadInitiate(ch, index, sub, size, timeout_ms, block)`,
    `CO_SDOclientDownloadBufWrite` (refilled from `data` while `CO_SDOclientDownload` reports a free buffer for
    larger writes);
  - `block` only for `*_data` transfers (`block_transfer_threshold`, e.g. > 32 bytes); value transfers stay
    expedited.
- Results: `CO_SDO_RT_ok_communicationEnd` → `SUCCESS` (+ width check); negative return with abort code
  `CO_SDO_AB_TIMEOUT` → `TIMEOUT`, other abort codes → `ABORT`; other errors → `TRANSPORT_ERROR`. A value read with
  more than 4 bytes → `SIZE_MISMATCH`.
- `CO_SDOclientClose(ch)`, free the slot, **then** the callback (so it may queue the next request).
- `response_timeout` (YAML, default 500 ms) = per-transfer timeout. Failures logged at WARN.

### `canopen_node.h`: `CANopenNode` base class for profile components
- `CANopenNode(CANopenComponent *parent, uint8_t node)`; codegen (`canopen.register_node()`, see "Optional parts")
  calls `parent->monitor_node(node, timeout)` before the hub's `setup()`, so a profile is always monitored and gets
  events without extra YAML.
- `void setup_canopen()` registers the hub callbacks for its node and calls the virtual hooks:
  `on_node_online()`, `on_node_offline()`, `on_node_nmt_changed(NMTState)`, `on_node_reset()` (default: nothing),
  `on_node_emcy(const Emergency &)`.
- Forwarders with the node id: `read_sdo<T>()`, `write_sdo<T>()`, `read_sdo_data()`, `write_sdo_data()`,
  `cancel_sdo()` (with `this` as default context), `send_nmt()`, `add_pdo_listener(cob_id, cb)` (a
  `add_frame_listener` with full mask), `send_pdo(cob_id, data, len)`.
- State: `is_online()`, `get_nmt_state()`, `is_operational()` (online and OPERATIONAL), `get_node_id()`.
- Not here: `configure_rpdo(n, cob_id, mapping[])` / `configure_tpdo(n, cob_id, transmission_type, inhibit,
  event_timer, mapping[])` (remote PDO config over SDO: disable COB-ID bit 31 → sub0 = 0 → mapping entries →
  sub0 = count → enable, one completion callback) come with `cia402`, their first user ([cia402.md](cia402.md)); they need nothing
  beyond the API above.

### `__init__.py`: YAML
- Hub: `response_timeout` (1ms..60s, default 500ms), `on_heartbeat` (`node_id`,
  `state`), `on_emcy` (`node_id`, `emergency`), plus step 1 options.
- Actions:
  - `canopen.sdo_write`: `id`, `node_id`, `index`, `subindex` (0), `value` (templatable), `size` (1/2/4),
    `on_success`, `on_error` (`result`, `abort_code`), the "value doesn't fit" check;
  - `canopen.nmt`: `id`, `node_id` (0..127), `command`.
- Final validation: `node_id` unique per `canbus_id` across `canopen:` blocks; a monitored node can't be our own
  node id.

### `test/stm32l4_canopen.yaml`
- A boot-time test `script` (lambdas calling the API and logging each `SDOReply`) against node 0x01 (an old
  esphome-canopen node on canopen-stack) and our own node.

### Docs
- `docs/canopennode.md`: the handoff state; "Layers" gets `types.h` / `canopen_node.h` as the profile API.
- CLAUDE.md: current step → this file. User docs for the options come with the switch-over (step 9).

## Verification
1. `esphome config test/stm32l4_canopen.yaml`, then `esphome run test/stm32l4_canopen.yaml` (repo root,
   `../esphomeplatformzephyr/venv/bin/esphome`).
2. Test script results (logged):
   - `read_sdo<uint32_t>(0x01, 0x1018, 1)` → SUCCESS `0xa59a08f5`; `read_sdo<uint16_t>(0x01, 0x1017, 0)` → 5000;
     `read_sdo<uint32_t>` of the u16 0x1017 → SIZE_MISMATCH;
   - `read_sdo_data(0x01, 0x1008, 0)` → the node's name (segmented); a long string through block upload;
   - read 0x01 0x5FFF → ABORT `0x06020000`; read from absent node 0x05 → TIMEOUT after ~500 ms;
   - `write_sdo<uint16_t>(0x01, 0x1017, 2000)` → SUCCESS, node 0x01's heartbeat period becomes 2 s (raw heartbeat
     timestamps); write back 5000;
   - local: `read_sdo<uint16_t>(0x7f, 0x1017)` → 1000, no frames on the bus;
   - queue: 20 reads → 16 run, 4 rejected with a synchronous REJECTED callback, each exactly once;
   - `cancel_sdo(ctx)` with 5 queued and one in flight → none of them calls back, the one in flight is aborted
     (`0x08000000` on the bus); a write queued right after starts next;
   - node monitoring: `monitor_node(0x01, 7000)` → ONLINE; `send_nmt(0x01, PRE_OPERATIONAL)` → NMT_CHANGED 127;
     START → 5; `RESET_NODE` → RESET then ONLINE; (unplug) → OFFLINE;
   - frame listener on node 0x01's TPDO COB-IDs (its entities, `0x181`...) → frames with data;
   - `send_frame()` on an unused ID (e.g. `0x37F`): no receiver on the bus, so check that it leaves the TX queue
     and that the bus stays error-active with 0 TX errors (frames are acked). Done 2026-10-29 for PR 1 (see
     `test/stm32l4_canopen.yaml`: TPDO1 listener caught node 0x32's `0x1b2`; overlap warning for 0x701; `send_frame`
     every 5 s ok, 0 TX errors);
3. CiA 402 feasibility: sketch the port of #21533's `cia402` onto this API in the scratchpad (not committed), at
   least `setup()` (PDO mapping of statusword/position into the drive's TPDOs, controlword into its RPDO via
   `configure_tpdo`/`configure_rpdo`, written in the sketch on top of `write_sdo`), the state machine on
   `add_pdo_listener` and `on_node_reset()` → `apply_configuration()`, and stop via the RPDO (`send_pdo`) and via
   `cancel_sdo()` + `write_sdo`. Compile it against `components_next` to check the API holds.
4. Optional parts: build `test/stm32l4_canopen.yaml` without anything that needs the SDO client / NMT master and
   with no `heartbeat_clients`, check in `compile_commands.json` that `-DCO_CONFIG_SDO_CLI=0` etc. reach
   CANopenNode, and compare flash/RAM (`esphome compile` summary, or the `.map`) with the full test config.
5. `pytest test/` for the old component (needs pytest installed; old component untouched).

## Status (2026-10-29)
SDO client + NMT master implemented in `components_next` and as PR 3 (`canopen-3-sdo`). Hardware test
`test/sdo_test.h` against node 0x61 (`stm32-can-node-1`, old canopen-stack component) and our own node: 24/24 pass
(typed / buffer reads, size mismatch, abort, timeout, remote write checked in the heartbeat period, local transfers,
queue limit, one callback per request, cancel queued / in flight, NMT pre-operational / start checked in the
heartbeats, both actions).

Lessons from the test runs:
- **canopen-stack 0x1017 bug:** the old component stores 0x1017 as a direct value (`CO_OBJ_D___RW`), and
  canopen-stack's heartbeat type reports size 0 when the value is 0 (`COTNmtHbProdSize` checks `obj->Data != 0`), so
  after writing 0 every 0x1017 read/write aborts with 0x08000020 until a reboot (an NMT reset doesn't restore it).
  Its 0x1017 reads also abort (0x06040043) at some values. Don't write 0x1017 = 0 to old nodes; the test measures the
  heartbeat period from the frames instead of reading 0x1017.
- **Cancel in flight:** CANopenNode sends the abort frame only if the client's TX buffer is free; otherwise the server
  stays in the transfer and rejects the next request (0x05040001). `abort_sdo_()` sends the abort frame itself in that
  case, and the next request to the node waits 50 ms.
- Not covered yet: block transfers (reads were short; no block write > 32 bytes to a writable domain), `RESET_NODE`
  (resets whatever runs on the node; tested once, works).

## Open points
- Node 0x01 in the tests: the write to its 0x1017 and the NMT commands (PRE_OPERATIONAL, START, RESET_NODE) change
  a real node on the bus. Confirm that's OK, or limit those tests to a node that may be reset.
- **More and pinned SDO channels (later, one PR; documented, not implemented).** One shared channel covers CiA 402.
  When several drives are polled over SDO, or something needs a reserved channel or non-default COB-IDs (a device's
  second SDO server, 0x1201+; non-standard devices; the old `csdo:` option, `setup_csdo(num, node_id, tx_id,
  rx_id)`), extend it like this, without changing the request API:
  - pool: `sdo_clients: N` (default 1) → `CNT_SDO_CLI = N`, OD 0x1280+i; the dispatcher fills every idle channel
    with the first queued request whose node has no transfer on another channel (a server handles one transfer at a
    time);
  - `SDOChannel *acquire_sdo_channel(uint8_t node, uint16_t tx_id = 0, uint16_t rx_id = 0)` (0 = `0x600 + node` /
    `0x580 + node`), called from codegen before `setup()`: takes one channel out of the pool for one owner; the
    pinned channels are added to `CNT_SDO_CLI` on top of `sdo_clients`, so the shared pool keeps its size;
  - `read_sdo(SDOChannel *, ...)` / `write_sdo(SDOChannel *, ...)` overloads: same `Request`, `SDOReply`, callback
    and `cancel_sdo()` rules, but the dispatcher runs them only on that channel (in order, one at a time);
  - YAML: `csdo: [{node_id, tx_id, rx_id}]` on the hub maps onto it.
- **Urgent requests (later, with the channel pool).** `enum class SDOPriority : uint8_t {NORMAL, URGENT}` as a last,
  defaulted parameter of `read_sdo`/`write_sdo` (no API break), `priority:` on `canopen.sdo_write`. `URGENT` goes to
  the front of the queue and aborts a `NORMAL` transfer to the same node (`ABORT`/`0x08000000` to its owner, which
  retries if it still needs it), else takes an idle channel. Only needed if `cancel_sdo()` + write turns out too
  slow, i.e. when other components keep the channel busy.
- `NMTState` value for "no heartbeat yet": `UNKNOWN = 255` vs CANopenNode's `CO_NMT_UNKNOWN = -1` (mapped in the
  core).
- PDO listeners are raw: the profile decodes bytes. If several profiles end up with the same decoding code, add a
  typed `PdoMapping` helper later.
- RAM: every SDO client channel has a 1000-byte FIFO (`CO_CONFIG_SDO_CLI_BUFFER_SIZE`); one channel in this step. Make the
  buffer size a YAML option if STM32 targets get tight.

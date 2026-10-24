# Remote ESPHome logging over CANopen

## Context
Nodes on the CAN bus (often STM32/Zephyr without WiFi) can only be debugged over a serial cable. Goal: stream
ESPHome log output over CAN to can2mqtt, which prints it in its own log. Decisions made:
- **Host polls**: node is a pure SDO server (it is built with `USE_CSDO=0`; canopen-stack's SSDO supports
  block upload, `co_ssdo.c` `COSdoInitUploadBlock`/`COSdoUploadBlock`). No new COB-IDs.
- **Host subscribes**: node buffers nothing until can2mqtt writes a log level; if nobody polls for `timeout`
  (default 30 s, configurable) it turns itself off again (similar to `esphome logs` subscribing via the API).
- **Per-node default level**: the node YAML declares a default level (`remote_logs: level:`, default `INFO`). The
  node then starts buffering at boot (so boot logs / `dump_config` are captured) and can2mqtt in `AUTO` mode
  subscribes with that level, so a bare `remote_logs: {}` means "send me logs". `level: NONE` makes the node wait
  for an explicit host subscription (`--remote-log-level <level>`). The node still never sends unsolicited frames.
- **Sink**: can2mqtt's own Python logging. Both repos are in scope.
- **Overload**: when logs are produced faster than they are read, the node drops the **newest** whole lines
  (never blocks, never overwrites) and marks the gap in the stream (see "Overflow" below).

## OD layout (new, 0x3100 "Logs"; entities end at 0x2FFF, firmware is 0x3000)
| Sub | Name | Type | Access | Notes |
|---|---|---|---|---|
| 1 | Log Data | DOMAIN | R | upload returns buffered bytes and **consumes** them; whole `\n`-terminated lines |
| 2 | Log Pending | UINT32 | R | bytes currently buffered; reading it counts as keepalive |
| 3 | Log Level | UINT8 | RW | 0 = off, 1..7 = ESPHome levels (ERROR..VERY_VERBOSE); write = subscribe + keepalive; the buffer is cleared only on a 0 → >0 transition (rewriting the same level as keepalive keeps it) |
| 4 | Log Dropped | UINT32 | R | messages dropped because the buffer was full (monotonic) |
| 5 | Log Default Level | UINT8 | R | from `remote_logs: level:` (0 = NONE); also the level the node starts with at boot |
| 6 | Log Timeout | UINT32 | R | auto-off timeout in ms from `remote_logs: timeout:` (0 = never) |

`:5` and `:6` are read-only on purpose: the node YAML is the single owner of these settings (no question of what
happens to a host-written value after a reboot).

Upload with size 0 makes canopen-stack abort (`COSdoGetSize`: size 0 → `CO_SDO_ERR_TOS`, in expedited,
segmented and block paths), so the host always reads `:2` first.

### How canopen-stack calls the DOMAIN type on upload (checked in `co_ssdo.c` / `co_obj.c`)
- Every upload calls `Size` first (`COSdoGetSize`), then `COObjReset` (expedited path directly;
  segmented/block via `COObjRdBufStart`, `co_obj.c:72`), then `Read` one or more times.
- Expedited (size ≤ 4): one `Read(len = size)`.
- Segmented: `Read` with `len` ≤ 7, capped at the remaining announced size.
- Block: `Read(len = SegNum * 7)` per block, and **the last block also passes the full `SegNum * 7`**, not the
  remaining size (`COSdoUploadBlock`, ~line 919). For block size ≤ 4 `RdBufStart` (and so `Reset`) is skipped.
- Block retransmits (`BLK_REPEAT`) resend from the SDO buffer, not from the object: consuming in `Read` is safe.

### Block size for log uploads
- The block size is negotiated per direction. **Download** (host → node, CAN OTA): the node announces it,
  `COSdoBlockSizeRequest(size, CO_SDO_BUF_SEG)` (`co_ssdo.c:592`), i.e. `sdo_block_transfer_size`. **Upload**
  (node → host, logs): the client requests it in the initiate request (byte 4) and again in every block ack; the node
  caps it at `CO_SDO_BUF_SEG` (`co_ssdo.c:829-838`, `:1002-1010`). So log uploads can use a small block size without
  touching OTA.
- **Zephyr TX limit**: `zephyr_can` sends with `can_send(K_NO_WAIT)` and has no software TX queue; the STM32 bxCAN
  and FDCAN drivers have 3 TX mailboxes/buffers. canopen-stack sends a whole upload block in a tight loop
  (`COSdoUploadBlock`, ~line 935) and ignores send errors (`(void)COIfCanSend`; our `DrvCanSend` returns 0 even when
  `send_data` fails). Every segment past the 3rd in a block is silently lost. Until `zephyr_can` gets a TX queue,
  log uploads use **block size 3** on all platforms (constant in can2mqtt; switch to 2 if `Will start
  retransmission` shows up, which means a heartbeat/TPDO was still occupying a mailbox).
- **canopen-python / canopen-stack mismatch** (affects any block upload from our nodes, not just logs):
  canopen-python's `BlockUploadStream` requests `blksize = 64` (class attribute; `sdo.open()` gives no way to change
  it). canopen-stack silently caps it at `CO_SDO_BUF_SEG` and the initiate response has no field to tell the client.
  The node then sends e.g. 16 segments and waits for an ack, the client waits for segment 17, times out, acks 16 with
  blksize 64, its `_ackseq` doesn't reset, and the next block's seq 1 looks out of order → "Some data were lost".
  So **the client must request a block size ≤ the node's `CO_SDO_BUF_SEG`**. can2mqtt uses a subclass:
  ```python
  class LogUploadStream(BlockUploadStream):
      blksize = 3   # Zephyr: 3 TX mailboxes; must also be <= node's CO_SDO_BUF_SEG
  ```
- Node validation: with `remote_logs:` present, require `sdo_block_transfer_size >= 3` (its range is 1..127),
  so the request of 3 is never capped.
- Throughput with block size 3: 3 data frames + 1 ack per 21 B, vs. 2 frames per 7 B segmented; the host round-trip
  time is the real limit. Plenty for logs.
- Accepted limitation: an aborted / timed-out transfer loses the bytes already consumed by `Read`.

## Node side (esphome-canopen)

### `components/canopen/__init__.py`
- New optional key in `CONFIG_SCHEMA`:
  ```yaml
  remote_logs:
    buffer_size: 2048   # int, 256..16384
    level: INFO         # NONE/ERROR/WARN/INFO/CONFIG/DEBUG/VERBOSE/VERY_VERBOSE (logger's level names)
    timeout: 30s        # cv.positive_time_period_milliseconds, 1s..1h, or "never" (-> 0)
  ```
- In `to_code()` when present: `cg.add_define("USE_CANOPEN_LOGS")`, `cg.add_define("CANOPEN_LOG_BUFFER_SIZE", n)`,
  `cg.add(canopen.set_log_default_level(lvl))`, `cg.add(canopen.set_log_timeout(ms))`, and call `esphome.components.logger.request_log_listener()` (exists in upstream and in the
  esphomeplatformzephyr fork; import lazily / `try ImportError` per CLAUDE.md compat rule). Without it
  `USE_LOG_LISTENERS` is undefined and `add_log_callback` is a silent no-op (`logger.h:187-193`). Validate that
  `logger` is in the config (`cv.requires_component("logger")` or a final_validate check).
- Validation: warn if `level` is more verbose than the logger's compile-time `logger: level:` (those messages are
  compiled out and would never arrive; with the INFO default this also fires for a bare `remote_logs: {}` on a
  `logger: level: WARN`/`ERROR` node, which is intended). Warn if `timeout: never` is used with `level: NONE` (harmless but pointless:
  nothing buffers until a host subscribes, and then the subscription never expires). Error if
  `sdo_block_transfer_size < 3` (see "Block size for log uploads").

### New `components/canopen/logs.h` / `logs.cpp` (pattern: `fw.h`/`fw.cpp`, guarded by `#ifdef USE_CANOPEN_LOGS`)
- `struct CanopenLogs` holding: `uint8_t buf[CANOPEN_LOG_BUFFER_SIZE]` (static, see "Footprint"), `head`, `tail`
  (ring buffer), `uint32_t pending`, `uint32_t dropped`, `uint32_t unreported_drops`, `uint32_t xfer_remaining`,
  `uint8_t level`, `uint8_t default_level`, `uint32_t timeout_ms`, `uint32_t last_poll_ms`.
- At setup: `level = default_level`, `last_poll_ms = millis()`, so with a default level the node buffers from the
  first line logged after the callback is registered, and the timeout counts from boot. Lines logged before
  `CanopenComponent::setup()` runs (earlier setup priorities) are not captured.
- `on_log(level, tag, message, len)`: return if `level == 0` or msg level > `level`; skip the `canbus` and
  `canopen` tags below INFO (`canbus` logs every frame at DEBUG; anything logged on the SDO/OD path while serving
  a log read would feed itself). Append `message` + `'\n'` only if the whole line fits, else `dropped++`,
  `unreported_drops++` (see "Overflow"). Messages longer than the buffer are truncated to fit.
- `on_log` must not log itself (the logger's `RecursionGuard` would drop it anyway). `Read`, `Reset`, `Write`
  must not call `ESP_LOG*` either: a log there would call `on_log` in the middle of a consume.
- Concurrency: **no mutex**. Log listeners are only ever called on the main loop, in upstream ESPHome and in the
  esphomeplatformzephyr fork alike: main-task logs go through `log_message_to_buffer_and_send_()` →
  `notify_listeners_()`; logs from other tasks are queued in the task log buffer and delivered from
  `Logger::process_messages_()` in the logger's `loop()` (without `USE_ESPHOME_TASK_LOG_BUFFER` they only go to the
  console). SDO processing (`on_frame` → `CONodeProcess`) also runs on the main loop. Re-check this when bumping
  ESPHome; if it ever changes, add the mutex back.
- CO_OBJ_TYPEs (same shape as `FwImage = {Size, Init, Read, Write, Reset}`):
  - `LogData`:
    - `Size`: `xfer_remaining = min(pending, buffer size)`, return it. The snapshot is taken here as well
      as in `Reset` because block uploads ≤ 4 bytes skip `Reset`.
    - `Reset`: `xfer_remaining = min(pending, buffer size)` again (same main-loop call as `Size`, with no logging
      in between, so the value is identical).
    - `Read(buf, len)`: `n = min(len, xfer_remaining)`; copy `n` bytes from `tail` (handling wrap), advance `tail`,
      `pending -= n`, `xfer_remaining -= n`, zero-fill `buf[n..len)`, set `last_poll_ms`. **Never consume more
      than announced**: the last block passes `len = SegNum * 7` (see above), and consuming that would eat lines
      appended after `Size` without ever sending them.
    - Data is only ever appended at `head`, so the announced bytes are still there when `Read` runs.
  - `LogPending`: read-only UINT32 whose `Read` returns `pending` and sets `last_poll_ms`.
  - `LogLevel`: UINT8 RW; `Write` stores level, clears buffer (and `unreported_drops`) only when going
    0 → >0, sets `last_poll_ms`.
  - `:4` dropped: plain `CO_TUNSIGNED32` with `CO_OBJ_____R_` pointing at `dropped`.
  - `:5` default level, `:6` timeout: plain constants, `CO_OBJ_D___R_` (`CO_TUNSIGNED8` / `CO_TUNSIGNED32`).
- Register the callback in `CanopenComponent::setup()` with
  `logger::global_logger->add_log_callback(this_logs, [](void *self, uint8_t lvl, const char *tag, const char *msg, size_t len){...})`
  (upstream API from `logger.h`), next to the 0x3000 block (`canopen.cpp:522`), and add the six
  `od.add_update(CO_KEY(0x3100, n, ...), ...)` entries there.

### Auto-off (`CanopenComponent::loop()`, `canopen.cpp:755`)
- `last_poll_ms` is set by: Log Pending read, every Log Data `Read`, any Log Level write. Not by reads of
  `:3`..`:6` or any other SDO/PDO traffic.
- Check: `if (level && timeout_ms && millis() - last_poll_ms > timeout_ms)` (unsigned subtraction survives
  `millis()` wrap) → `level = 0`, `head = tail = pending = unreported_drops = 0`; then, after the reset (so it
  goes to serial only), `ESP_LOGI(TAG, "remote logging disabled (no poll for %u ms)", timeout_ms)`. `dropped` is
  kept (since boot).
- After auto-off the node stays at 0 until a host writes a level, even if `default_level` is set: the default
  applies at boot only.
- `timeout: never`: the subscription never expires. After the buffer fills, `on_log` only counts drops (cheap, not
  free). Mainly useful with a default level, so boot logs wait until someone reads them.

### Overflow (logs produced faster than read)
- Policy: drop the **newest** whole line; never block the logger, never overwrite unread data. This keeps the
  invariant that bytes announced by `Size` stay valid during a transfer (`on_log` only moves `head`, `Read`
  only moves `tail`). Serial logging and the main loop are unaffected.
- Gap marker: while `unreported_drops > 0`, `on_log` first tries to append the marker line
  `[W][canopen.logs]: <N> messages dropped\n` (preformatted, no `ESP_LOG*`). The current line is appended after
  it only if marker + line fit; otherwise both are skipped and the counters grow. Once the marker is in,
  `unreported_drops = 0`. This puts the gap exactly where it happened in the stream; `Log Dropped` stays as the
  monotonic cross-check.
- Throughput: bounded by the SDO block upload rate on the bus (a few KB/s; less via lossy bridges), provided the
  host drains in a loop (below). Sustained overload is handled by a lower `--remote-log-level` or per-tag
  `logger: logs:` levels; `buffer_size` absorbs bursts (boot, `dump_config`).

### Footprint (flash / RAM)
**`remote_logs:` not in the YAML: zero cost**, byte-identical firmware to a build without the feature:
- everything node-side is behind `#ifdef USE_CANOPEN_LOGS`: `logs.h`, the whole body of `logs.cpp`, the
  `CanopenLogs` member and setters in `canopen.h`, the OD entries and callback registration in `setup()`, the
  check in `loop()`. No stubs, no empty virtuals, no unconditional members;
- `request_log_listener()` is only called when the key is present. Otherwise `USE_LOG_LISTENERS` /
  `ESPHOME_LOG_MAX_LISTENERS` are untouched (still set if other components such as `api` request listeners), so the
  logger's listener `StaticVector` and dispatch loop are not added or enlarged by us;
- the OD is the fixed `object_dictionary[APP_OBJ_N]` array filled at runtime, so no change there;
- the level-name table, validation and EDS entries live in Python / can2mqtt only.

**`remote_logs:` present:**
- RAM: the ring buffer `CANOPEN_LOG_BUFFER_SIZE` (default 2048) as a **static** array in `.bss`. It shows up in
  ESPHome's build "RAM:" summary and in `size firmware.elf`, never fails at runtime and doesn't fragment the heap
  (matters on STM32/Zephyr, where heap is small). Plus ~30 B of state (`head`, `tail`, counters, levels,
  timestamps; use `uint16_t` for `head`/`tail` since the buffer is ≤ 16384) and one listener slot (8 B) in the
  logger. No mutex (see Concurrency).
- RAM in the OD: 7 entries in the existing fixed array (no growth unless it's full; `APP_OBJ_N` stays 512).
- Flash: target < 1.5 KB. `on_log` + ring copy, three small `CO_OBJ_TYPE`s, the timeout check, the marker string.
  Format the marker count by hand (small `utoa`), not with `snprintf`, in case nothing else pulls in printf
  (on ESPHome it usually is already linked, but don't rely on it).
- `buffer_size` lower bound stays 256 B for very small nodes.
- Not considered: lazy allocation on first subscribe would save the buffer on nodes that are never subscribed, but
  moves a possible failure to runtime and fragments the heap. Users who don't want the RAM cost leave
  `remote_logs:` out.

### `canopen.h`
- `#ifdef USE_CANOPEN_LOGS`: `set_log_default_level()`, `set_log_timeout()`. The state is the global
  `CanopenLogs canopen_logs` in `logs.cpp`, not a member: the component is heap-allocated (`cg.new_Pvariable`),
  and a global keeps the ring buffer in `.bss`. Hence `remote_logs:` on one `canopen` node per firmware only.
- Bump `APP_OBJ_N` only if needed (7 more objects incl. sub0; currently 512).

### Docs
- `OBJECT_DICTIONARY.md`: new "Logs (0x3100)" section with the table above, incl. "clear only on 0 → >0" and
  the consume-on-read semantics (aborted transfers lose data).
- `README.md`: `remote_logs:` options (`buffer_size`, `level`, `timeout`), note to set
  `logger: logs: canbus: INFO`; overflow behaviour (gap marker, `buffer_size` for bursts, lower the level for
  sustained load); timeout trade-offs (short: stops quickly when the host is gone but an OTA / busy host turns it
  off sooner; long: survives a can2mqtt restart up to `buffer_size`; `never`: see Auto-off).
- `CLAUDE.md`: add `logs.cpp/.h` to Layout and a short "Remote logs" flow note (snapshot in `Size`/`Reset`,
  `Read` capped at the snapshot, drop-newest + marker).
- `test/config.yaml.template`: add `remote_logs: {level: INFO, timeout: 60s}` so the Docker build covers the
  options.

## Host side (~/repos/can2mqtt)
- `src/can2mqtt/eds/esphome.eds`: add `[3100]` record "Logs" with subs 0..6 ("Log Data" DOMAIN R, "Log Pending"
  UINT32 R, "Log Level" UINT8 RW, "Log Dropped" UINT32 R, "Log Default Level" UINT8 R, "Log Timeout" UINT32 R),
  following the `[3000]` Firmware entries.
- `src/main.py`: new `--remote-log-level` (AUTO/NONE/ERROR/WARN/INFO/CONFIG/DEBUG/VERBOSE/VERY_VERBOSE, default
  `AUTO`), passed through `config` to `start()`; `config.yaml` addon option `remote_log_level: str?`.
  - `AUTO`: per node, subscribe with the node's `:5` Log Default Level; 0 → leave the node alone.
  - `NONE`: never subscribe (feature off globally).
  - explicit level: use it for every node that has 0x3100, ignoring `:5`.
- `src/can2mqtt/can2mqtt.py`:
  - In `process_node()` (near the Firmware Flags probe at ~line 194), probe `node.sdo["Logs"]["Log Level"]`
    in try/except (old firmware → no logs, debug message). If supported, read `:5` and `:6` (each in try/except:
    missing → default level 0, timeout 30000 ms) and resolve the level per the mode above. If the resolved
    level is not 0, cancel any previous `node.log_task` and start
    `node.log_task = asyncio.create_task(log_reader(node, level, timeout_ms))`.
  - Intervals derived from the node timeout: keepalive `= min(10 s, timeout / 3)` (two lost writes still don't
    expire the subscription), Pending poll `= min(1 s, timeout / 3)`; `timeout = 0` (never) → 10 s / 1 s.
  - `log_reader(node, level, timeout_ms)`: loop forever:
    - every keepalive interval (and on start) `aset_raw(level)` on Log Level: keepalive and re-subscribe after
      node reboot or auto-off (same level → node keeps its buffer, including boot logs buffered under the
      default level);
    - every poll interval read Log Pending; if >0 read Log Data: `aget_data`/`aupload` (segmented/expedited) for small
      amounts, block upload for larger ones (> ~64 B) via `async with node.sdo.lock: await asyncio.to_thread(lambda:
      LogUploadStream(node.sdo, 0x3100, 1, request_crc_support=False).readall())` (block size 3, see "Block size
      for log uploads"; not `sdo.open(..., block_transfer=True)`, which always requests 64; same lock `aupload`
      uses, `sdo/client.py:145`). The node never does CRC (`0xC2`), so CRC isn't requested;
    - **drain loop**: after a Log Data read, re-read Log Pending and read again right away while it is > 0
      (bounded, e.g. max 16 reads, yielding with `await asyncio.sleep(0)` in between), so the bus speed, not the
      1 s poll interval, limits throughput;
    - decode utf-8 (`errors="replace"`), strip ANSI `\x1b\[[0-9;]*m`, split lines, map ESPHome prefix
      `[E]/[W]/[I]/[C]/[D]/[V]` to Python levels and log through `logging.getLogger(f"node.{node.name or node_id}")`
      (the node's `[W][canopen.logs]: N messages dropped` marker comes through as a normal warning);
    - read Log Dropped occasionally, warn on increase (cross-check for the in-stream marker);
    - skip polling while a firmware upload to that node is running (flag set/cleared in `firmware_upload()`); the
      node then turns logging off after its timeout, which is fine: it reboots after OTA, and after a failed OTA
      the next keepalive re-subscribes;
    - SDO errors/timeouts: log at debug, back off 5 s, continue; task is cancelled when the node is reloaded.
  - If can2mqtt restarts within the node timeout with a different level, lines already buffered at the old level
    are still delivered; harmless. Same when an explicit `--remote-log-level` differs from the node's default level:
    boot lines buffered at the default level arrive first.

## Verification
1. `esphome config` on a yaml with `remote_logs: {}` and with `remote_logs: {level: INFO, timeout: never}` in both
   upstream ESPHome and the esphomeplatformzephyr venv; also without `logger:` (validation error), and with
   `level: VERBOSE` on a `logger: level: DEBUG` node (warning).
2. `./docker_build.sh` (ESP32) builds; Zephyr example `zephyr-examples/stm32l4/stm32l4_blinking_led.yaml` with
   `remote_logs:` builds; check `compile_commands.json` that `USE_CANOPEN_LOGS` and `USE_LOG_LISTENERS` reach
   `canopen.cpp`/`logs.cpp`.
2a. Footprint: build the ESP32 test config and the Zephyr example (a) at the commit before the feature,
   (b) with the feature but without `remote_logs:`, (c) with `remote_logs: {}`. (a) and (b) must have identical
   `size firmware.elf` text/data/bss (ideally identical `.bin`); (c) − (b): bss ≈ `buffer_size` + < 100 B,
   text < 1.5 KB. Record the numbers in the PR description.
3. Hardware: run can2mqtt with `--remote-log-level DEBUG`; node logs appear in can2mqtt output with correct levels;
   compare with the serial log.
4. Stop can2mqtt → after ~30 s node logs "remote logging disabled" on serial and stops buffering. Repeat with
   `timeout: 5s` (off after ~5 s; can2mqtt keepalive every ~1.7 s keeps it on while running) and `timeout: never`
   (never turns off).
4a. Boot-log capture: node with `level: INFO`, can2mqtt in `AUTO` → after a node reset, boot / `dump_config` lines
   appear in can2mqtt. Node with `level: INFO` and can2mqtt not running → "remote logging disabled" on serial ~30 s
   after boot. `--remote-log-level NONE` → no subscription even for nodes with a default level; node with `level: NONE` and
   `AUTO` → no log task; node with a bare `remote_logs: {}` and `AUTO` → subscribed at INFO.
5. Small `buffer_size` + log spam (e.g. `interval` lambda logging) → `N messages dropped` marker appears in the
   can2mqtt output at the gap, Log Dropped grows, can2mqtt warns, no crash, serial log unaffected.
6. Last-partial-block case: log continuously while can2mqtt does block uploads of sizes that are not a multiple of
   `SegNum * 7`; compare against the serial log: no lines missing or duplicated except at reported drop markers.
7. Drain loop: a burst larger than one read (e.g. `dump_config` at boot with `buffer_size: 4096`) arrives within
   one poll cycle, not spread over several seconds.
6a. Zephyr (STM32 bxCAN) node with `sdo_block_transfer_size: 16`: log block uploads (block size 3) complete without
   `Will start retransmission` in can2mqtt's DEBUG log; CAN OTA still uses block size 16 and its usual speed.
   `esphome config` with `remote_logs:` and `sdo_block_transfer_size: 2` fails validation.
8. Run a CAN OTA with logging on: upload still completes; node reboots, can2mqtt re-subscribes.
9. Node with old firmware (no 0x3100): can2mqtt starts normally, no log task.

## Appendix: protocol walkthrough

Assumptions: node id `0x05` (client → node `0x605`, node → client `0x585`), log upload block size 3 (requested by
can2mqtt; the node's `sdo_block_transfer_size` is larger and only used for OTA), `buffer_size` 2048, can2mqtt `--remote-log-level DEBUG`. Index 0x3100 is sent
little-endian as `00 31`; all multi-byte values are little-endian.

The log callback gets each line with ANSI colors (`\x1b[0;36m` … `\x1b[0m`, +11 bytes); the node appends `\n`.

| Line | Time | Text (without ANSI) | Bytes in buffer |
|---|---|---|---|
| L1 | 0.30 s | `[D][sensor:093]: 'Temp' 21.5` | 40 |
| L2 | 1.40 s | `[W][dht:120]: timeout` | 33 |
| L3 | 1.80 s | burst of 4 `[C]` lines (`dump_config` of a component) | 150 |
| L4 | 2.01 s | `[D][sensor:093]: 'Temp' 21.6` | 40 |

### t = 0.00: node found, subscribe
Probe whether the node supports logs (expedited upload of Log Level):
```
605: 40 00 31 03 00 00 00 00     upload request 0x3100:3
585: 4F 00 31 03 00 00 00 00     expedited reply, 1 byte: level = 0
```
Old firmware answers with an abort (`80 …`, object does not exist): can2mqtt logs at debug, no log task.

Then the node's settings (this node is configured with `level: NONE`; timeout is the default 30 s):
```
605: 40 00 31 05 00 00 00 00     read Log Default Level
585: 4F 00 31 05 00 00 00 00     0 (NONE; with AUTO this node would not be subscribed, DEBUG is explicit here)
605: 40 00 31 06 00 00 00 00     read Log Timeout
585: 43 00 31 06 30 75 00 00     30000 ms → keepalive 10 s, poll 1 s
```
(With `level: INFO` in the node YAML (the default) the first read above would return `03`, the node would already be buffering
since boot, and the Level write below would not be a 0 → >0 transition, so the boot lines are kept.)

Subscribe by writing level 5 (DEBUG):
```
605: 2F 00 31 03 05 00 00 00     expedited download, 1 byte
585: 60 00 31 03 00 00 00 00     ok
```
Node: `LogLevel.Write`, level goes 0 → 5, so the buffer is cleared and `last_poll_ms = now`. From now on
`on_log` stores lines at DEBUG or more severe.

### t = 0.30: L1 is logged
`on_log` appends 40 bytes at `head`, `pending = 40`. No CAN traffic: the node only answers requests.

### t = 1.00: first poll, small amount → segmented upload
```
605: 40 00 31 02 00 00 00 00     read Log Pending
585: 43 00 31 02 28 00 00 00     4 bytes: 40
```
Reading Log Pending counts as keepalive (`last_poll_ms = now`). 40 ≤ 64, so a plain `aupload`:
```
605: 40 00 31 01 00 00 00 00     upload request 0x3100:1
585: 41 00 31 01 28 00 00 00     segmented, size 40      <- Size() = 40, Reset() snapshots xfer_remaining = 40
605: 60 00 00 00 00 00 00 00     segment request, toggle 0
585: 00 1B 5B 30 3B 33 36 6D     7 bytes "\x1b[0;36m"   <- Read(7), consumes 7
605: 70 …                        toggle 1
585: 10 5B 44 5D 5B 73 65 6E     7 bytes "[D][sen"
   … (6 segments in total: 5×7 + 5)
585: 15 1B 5B 30 6D 0A 00 00     last: toggle 1, 2 unused bytes, c=1; "\x1b[0m\n"
```
Data is consumed segment by segment as can2mqtt requests each one. If the client gives up halfway, the bytes
already read are gone (accepted limitation).

Drain loop: can2mqtt reads Log Pending again → 0 → waits for the next 1 s poll.

### t = 1.40 and 1.80: L2 and L3 arrive
`pending = 33`, then `183`.

### t = 2.00: larger amount → block upload
```
605: 40 00 31 02 …               read Log Pending
585: 43 00 31 02 B7 00 00 00     183
```
183 > 64 → block upload, holding `node.sdo.lock`:
```
605: A0 00 31 01 03 00 00 00     initiate block upload, no CRC requested, blksize 3, pst 0
585: C2 00 31 01 B7 00 00 00     no CRC, size 183         <- Size() = 183, Reset() snapshots xfer_remaining = 183
605: A3 00 00 00 00 00 00 00     start
```
Block 1: the node fills its SDO buffer with `Read(len = 3×7 = 21)`, which consumes 21 bytes
(`xfer_remaining = 162`), then sends the 3 segments back to back. They fit in the 3 TX mailboxes on STM32:
```
585: 01 <7 bytes>
585: 02 <7 bytes>
585: 03 <7 bytes>
605: A2 03 03 00 …               ack: last good seq 3, next blksize 3
```
If a frame had been lost, the ack would carry a lower seq. The node resends from its SDO buffer without calling
`Read` again, so nothing is consumed twice.

Blocks 2..8 are the same: each one `Read(21)` + 3 segments + ack. 183 = 8×21 + 15.

t = 2.01, around block 5: L4 is logged. `on_log` appends 40 bytes at `head`, `pending` grows by 40.
`xfer_remaining` doesn't change: this transfer announced 183 bytes and sends exactly that.

Block 9: canopen-stack again calls `Read(len = 21)` although only 15 bytes are left. `Read` takes
`n = min(21, 15) = 15` and zero-fills the rest. Without the cap it would also consume the first 6 bytes of L4,
which can2mqtt would never receive.
```
585: 01 <7 bytes>
585: 02 <7 bytes>
585: 83 <1 byte> 00 00 00 00 00 00   seq 3, bit 7 = last segment (15 = 2×7 + 1)
605: A2 03 03 00 …               ack seq 3
585: D9 00 00 00 00 00 00 00     end block: 6 unused bytes in last segment, CRC 0 (no CRC agreed)
605: A1 00 00 00 00 00 00 00     end ack
```
In total: 27 data segments + 9 acks + initiate/start/end frames, about 41 frames for 183 B.
Drain loop: can2mqtt reads Log Pending again → 40 (L4) → segmented upload as at t = 1.00 → Pending 0 → back to
the 1 s poll.

can2mqtt then processes the received data (L2, the four L3 lines, L4):
1. decode utf-8 and strip the ANSI codes;
2. split into lines;
3. map the level letter: `[W]` → WARNING, `[C]` → INFO, `[D]` → DEBUG;
4. log through `logging.getLogger("node.<name>")`.

### Idle, t = 3, 4, 5, …
One request/reply pair per second:
```
605: 40 00 31 02 …
585: 43 00 31 02 00 00 00 00     0 → no Log Data read (a size-0 upload would abort anyway)
```

### t = 10, 20, …: keepalive / re-subscribe
can2mqtt writes `2F 00 31 03 05 …` again. The level stays 5, so the buffer is not cleared. If the node rebooted
in between, its level is 0 again and this write re-subscribes it, starting with an empty buffer.

### Variations
- **Overflow** (can2mqtt busy, e.g. an OTA to another node, and 2 KB pile up):
  1. Each line that doesn't fit is refused whole: `dropped++`, `unreported_drops++`.
  2. After the next read frees space, the next line `on_log` sees is preceded by
     `[W][canopen.logs]: 17 messages dropped`.
  3. can2mqtt shows the start of the burst, then that warning exactly at the gap, then the newer lines.
  4. Reading `0x3100:4` now and then (`43 00 31 04 <u32>`) gives the monotonic total for the host-side warning.
- **can2mqtt goes away**: no Pending/Data reads and no level writes, so `last_poll_ms` stops updating. After
  30 s `loop()` sets level 0, clears the buffer and logs `remote logging disabled` on serial only.
- **Line arrives between the Pending read and the Data upload** (Pending says 40, L5 is logged before the upload
  request): `Size` announces 80 and can2mqtt receives both lines. The client always goes by the size in the
  upload response; Pending is only used to choose segmented or block transfer.

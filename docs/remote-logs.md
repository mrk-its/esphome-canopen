# Remote logging over CAN
Nodes without WiFi can send their ESPHome log output over CAN; can2mqtt polls it and prints it in its own log
(as `node.<name>`):
```yaml
logger:
  logs:
    canbus: INFO  # canbus logs every frame at DEBUG

canopen_entities:
  remote_logs:
    level: INFO
```
* `buffer_size` (Optional, int, 256..16384, default=2048): RAM buffer for log lines (static, shows up in the build's
  RAM summary).
* `level` (Optional, default=INFO): `NONE`, `ERROR`, `WARN`, `INFO`, `CONFIG`, `DEBUG`, `VERBOSE` or `VERY_VERBOSE`.
  The node records at this level from boot, all the time, whether a host reads or not (a flight recorder, see
  below). can2mqtt reads a node's logs once a level is chosen with the node's **Remote log level** select in Home
  Assistant (or always, at this level, with `can2mqtt --remote-log-level AUTO`); it first gets what the node
  recorded before, then the live lines. A host's level applies while it polls; NONE in the select (Log Level 0) or
  no poll for `timeout` returns the node to this level. With `NONE` nothing is recorded until a host sets a level.
  Levels more verbose than `logger: level:` are compiled out (the build warns).
* `timeout` (Optional, time 1s..1h or `never`, default=30s): without a poll for this long, a level set by a host
  returns to `level:` (and an upload the host abandoned is given up). The recorded lines are kept. `never`: a host's
  level stays until it writes another one or the node reboots.
* `block_size` (Optional, int, 1..127, default=3): frames per SDO block of a log upload (can2mqtt requests it). The
  node sends a block back to back, so it must fit the canbus TX queue next to frames that may already be waiting (up
  to 8 TPDOs and a heartbeat); frames that don't fit are lost and each loss costs an SDO timeout (2 s) and a
  retransmit. The default fits CAN controllers without a software queue (3 TX mailboxes). With `zephyr_can`'s
  `tx_buffer_size: 16` use about 7, with 32 about 23.

Requires `logger:`. Without `remote_logs:` the feature costs nothing (no code, no RAM). Remote logs use SDO block
transfer, which `canopen_entities` turns on (with `sdo_block_size: 16` if `canopen:` doesn't set it).

The buffer keeps the most recent lines: when it is full, the oldest lines are evicted, and the next upload starts
with `[W][canopen.logs]: N messages dropped`. A host reading after a while (e.g. after choosing a level in Home
Assistant) gets the last `buffer_size` bytes the node logged, so a node that logs little at `level:` (INFO on a
quiet node) still has its boot logs; a chatty one has only its recent past. Recording costs a copy of each
formatted line into the static buffer. For sustained load while a host reads, a larger `buffer_size` absorbs
bursts, or lower the level (or per-tag `logger: logs:` levels). The OD objects are described in
[OBJECT_DICTIONARY.md](../OBJECT_DICTIONARY.md#logs). The protocol design is in [can-logs-plan.md](can-logs-plan.md).

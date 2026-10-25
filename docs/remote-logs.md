# Remote logging over CAN
Nodes without WiFi can send their ESPHome log output over CAN; can2mqtt polls it and prints it in its own log
(as `node.<name>`):
```yaml
logger:
  logs:
    canbus: INFO  # canbus logs every frame at DEBUG

canopen:
  remote_logs:
    level: INFO
```
* `buffer_size` (Optional, int, 256..16384, default=2048): RAM buffer for log lines (static, shows up in the build's RAM summary)
* `level` (Optional, default=INFO): `NONE`, `ERROR`, `WARN`, `INFO`, `CONFIG`, `DEBUG`, `VERBOSE` or `VERY_VERBOSE`. The node buffers at this level from boot (so boot logs and `dump_config` are kept until can2mqtt reads them) and can2mqtt subscribes with it. With `NONE` the node waits until a host subscribes with an explicit level (`can2mqtt --remote-log-level DEBUG`). Levels more verbose than `logger: level:` are compiled out.
* `timeout` (Optional, time 1s..1h or `never`, default=30s): the node turns remote logging off when no host polled for this long. Short: stops buffering soon after can2mqtt goes away, but a busy host (e.g. a long OTA to another node) turns it off sooner. Long: survives a can2mqtt restart (up to `buffer_size` of lines). `never`: the subscription never expires.
* `block_size` (Optional, int, 1..127, default=3): frames per SDO block of a log upload (can2mqtt requests it). The node sends a block back to back, so it must fit the canbus TX queue next to frames that may already be waiting (up to 8 TPDOs and a heartbeat); frames that don't fit are lost and each loss costs an SDO timeout (2 s) and a retransmit. The default fits CAN controllers without a software queue (3 TX mailboxes). With `zephyr_can`'s `tx_buffer_size: 16` use about 7, with 32 about 23. At most `sdo_block_transfer_size`.

Requires `logger:`. Can be set on one `canopen` node per firmware. Without
`remote_logs:` the feature costs nothing (no code, no RAM).

When logs are produced faster than they are read, the newest lines are dropped and the gap is marked in the stream
with `[W][canopen.logs]: N messages dropped`. A larger `buffer_size` absorbs bursts; for sustained load lower the
level (or per-tag `logger: logs:` levels). The OD objects are described in [OBJECT_DICTIONARY.md](../OBJECT_DICTIONARY.md#logs).
The protocol design is in [can-logs-plan.md](can-logs-plan.md).

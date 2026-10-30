# CANopenNode step 3: SDO server and identity

## Status (2026-10-30): done as upstream PR 7
Implemented in `~/repos/esphome` on branch `canopen-next` (on top of `canopen-4-profile`; the branch for
all remaining hub PRs, cut into separate PR branches later). Differs from the plan below, which was written for the
old combined component:
- generic CiA 301, nothing esphome-canopen specific: `identity:` (`vendor_id`, `product_code`, `revision_number`,
  `serial_number`, default 0) instead of the fixed ESPHome vendor/product, `device_name` (default: the node's name),
  `hardware_version` (0x1009 only if set), `software_version` (default `ESPHOME_VERSION`), with setters
  (`set_identity()`, `set_device_name()`, ...) so `canopen_entity_mapper` sets the ESPHome values and can2mqtt's
  version string;
- the SDO server is always on (CiA 301), segmented only with CANopenNode's 32-byte buffer (port header defaults
  `CO_CONFIG_SDO_SRV` / `_BUFFER_SIZE`); block transfer (buffer >= 900) comes with OTA / the block size option;
- `ObjectDictionary::add_string(index, const char *)`: read only, no `ODA_STR`, no `ODA_MB`; empty string = one zero
  byte.

Tests: Python codegen and defaults, gtest `od_test.cpp` (strings, records, `OD_set_u32` on 0x1200 as CANopenNode
does), the 4-platform build test, `prek`. Hardware (nucleo, `test/stm32l4_canopen.yaml`): `dump_config` shows identity
and strings; local reads of 0x1008/0x1009/0x100A/0x1018:1 OK; another node on the bus (most likely can2mqtt)
read 0x1018:1..3 from our server right after boot (seen by a frame listener on 0x67F; each read follows the reply
to the previous one) and then stopped, as can2mqtt does for non-ESPHome vendors; `sdo_test` still 24/24. Flash
+672 B, RAM +128 B, CANopenNode heap 1932 B.
Not covered: segmented uploads and writes from a remote client (needs python-can via the bridge or can2mqtt with
matching identity).

## Context
Step 1 ([minimal_canopennode.md](minimal_canopennode.md)) gave `components_next/canopen` with NMT, heartbeat
producer/consumer, the driver and a runtime OD with VAR and ARRAY entries. Step 2
([device_api_canopennode.md](device_api_canopennode.md)) adds the SDO client, NMT master, events and `add_record()`.
Step 3 makes the node readable over SDO the way can2mqtt
discovers nodes:
- `register_node()` reads `Identity` (0x1018:1..3) and skips the node unless vendor/product match ESPHome's;
- `process_node()` reads `SoftwareVersion` (0x100A), `DeviceName` (0x1008), `HardwareVersion` (0x1009, abort is
  OK), `ProducerHeartbeatTime` (0x1017), then `Firmware` (0x3000) and `EntityTypes` (0x2000/0x2001), which don't
  exist yet.

This step also proves segmented and block transfers through our driver (expedited only needs single frames, block
transfer needs bursts of up to `sdo_block_transfer_size` frames processed in `on_frame_`), and the block size the
node announces. `sdo_block_transfer_size` is dropped for now (see "Block size" in [canopennode.md](canopennode.md)),
so CANopenNode announces 127: more than `zephyr_can` can buffer (64 RX frames). This step decides whether to bring
the option back (CANopenNode branch `sdo-srv-blksize`) or to live with 127 for now.

Out of scope: OD extensions (`OdHandler`), 0x1010/0x1011, logs, OTA, entities, PDOs.

## Changes

### `od.h` / `od.cpp`
- `add_record()` exists since step 2 (0x1280); 0x1018 and 0x1200 use it.
- `add_string(index, const std::string &)`: read-only VAR, data = the string's bytes, `len = size()`, **no
  `ODA_STR`**. With `ODA_STR` the SDO server doesn't indicate the size in the upload initiate (it only knows it at
  the terminator); without it the size is exact, strings of ≤ 4 bytes go expedited, longer ones segmented/block.
  The component keeps the strings (`std::string` members) so the pointers stay valid. An empty string → one zero
  byte (a zero-length upload is aborted with `CO_SDO_AB_DEVICE_INCOMPAT`).

### `canopen.h` / `canopen.cpp` (`CANopenComponent` since step 2)
- New OD entries in `setup()`:
  - `0x1008` device name: `App.get_name()`;
  - `0x1009` hardware version: only if `hw_version` is set (can2mqtt treats the abort as "none");
  - `0x100A` software version: see codegen below;
  - `0x1018` identity RECORD: sub0 = 4 (u8, R), vendor `0xa59a08f5`, product `0x6bdfa1d9`, revision 1, serial 0
    (u32, R), constants from the old `components/canopen/canopen.cpp:97`;
  - `0x1200` SDO server 1 RECORD: sub0 = 2, sub1 `0x600 + node_id`, sub2 `0x580 + node_id` (u32, R).
    `CO_SDOserver_init()` recognises index 0x1200 and fills subs 1/2 with the default COB-IDs itself
    (`OD_set_u32(..., true)`, original data, no attribute check), so the storage just needs to exist.
- `config_`: `CNT_SDO_SRV = 1`, `ENTRY_H1200 = od_.find(0x1200)`.
- `CO_CANopenInit(... SDOserverTimeoutTime_ms = 1000 ...)` already passes the server timeout; keep it.
- Setters: `set_hw_version(const char *)`, `set_sw_version(const char *)`.
- 0x1017 is already RW: CANopenNode installs its own extension on it (`CO_NMT_init`), so an SDO write changes the
  heartbeat period live. That's checked below, no code needed.

### `__init__.py`
- Options `hw_version` (string, optional) and `sw_version` (string, optional), as in the old schema.
- Keep the old version string exactly, since can2mqtt's `firmware_scanner.py` finds the firmware version for a
  node by searching the binary for it (`VER_RE`):
  `const char *version_str_<node_id> = "node_id: <id:02x>, ver: " ESPHOME_VERSION ".<UTC timestamp>";`
  and 0x100A = `version_str_<node_id> + 18` unless `sw_version` is given (same as the old `to_code`,
  `components/canopen/__init__.py:615`).

### `test/stm32l4_canopen.yaml`
- Add `hw_version: nucleo_l476rg` to exercise 0x1009.

### Docs
- `docs/canopennode.md` "Current state (handoff)": step 3 status. CLAUDE.md: current step → this file.

## Verification
1. `../esphomeplatformzephyr/venv/bin/esphome config test/stm32l4_canopen.yaml`, then `run`; boot log as in step 1,
   plus the SDO server initialised (no `CO_CANopenInit` error, `errInfo` would name 0x1200 or 0x1018).
2. From the host, with the esp_usb_serial bridge on the bus (`can2mqtt/testcan.sh`: python-can interface
   `esp_usb_serial`, channel = the bridge's JTAG serial port), a throwaway script in the scratchpad using the
   can2mqtt venv (`canopen` / canopen-asyncio):
   - expedited: 0x1000, 0x1018:1..4 (vendor/product must match), 0x1200:1/2 = 0x67F/0x5FF, 0x1017;
   - segmented: 0x1008 (`stm32l4-blinking-led`), 0x100A (`<ESPHOME_VERSION>.<timestamp>`), 0x1009;
   - block upload: 0x100A with `block_transfer=True`;
   - block download: write 0x1017 with `block_transfer=True` and check the blksize the server announces in the
     initiate response (127 without the option; whether a 127-frame download survives `zephyr_can`);
   - write 0x1017 = 2000 → heartbeat on 0x77F every 2 s (seen in the script), write back 1000;
   - unknown index (e.g. 0x2000) → abort `0x06020000` (object does not exist), node keeps running.
3. can2mqtt against the node: it should register node 0x7f (identity OK), read the strings, then stop at
   0x3000/0x2000 (missing). The rest of discovery comes with the later steps.
4. Old component untouched (`git diff --stat components/` empty).

## Open points
- Host-side test needs the esp_usb_serial bridge connected; only the nucleo's ST-Link (`/dev/ttyACM0`) was
  attached during step 1.
- `CO_CONFIG_SDO_SRV_BUFFER_SIZE` stays 900 (needed with block transfer); RAM is fine on the L476 (128 KB). Sizing it
  from YAML is a later question (see "Open questions" in canopennode.md).

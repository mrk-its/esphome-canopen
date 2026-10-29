# Minimal CANopenNode-based `canopen` component (heartbeat only)

## Context
We're replacing canopen-stack with CANopenNode. The first step is a minimal, separate `canopen` component on top
of ESPHome `canbus`. It uses the local CANopenNode fork (`~/repos/CANopenNode`, branch `esphome`, commit `dce5a01`:
`library.json`, `port/esphome/CO_driver_target.h`, `CO_CONFIG_SDO_SRV_BLKSIZE`) and only does NMT + heartbeat:
- produce heartbeats (0x700+node_id);
- consume heartbeats of configured nodes (`heartbeat_clients`) through CANopenNode's HB consumer.

This proves the hard parts: the library pulled in through the converter, the driver glue, the OD built at runtime
(`CO_MULTIPLE_OD`), and `CO_process` timing. It builds for `test/stm32l4_canopen.yaml` (Zephyr, nucleo_l476rg)
with the esphomeplatformzephyr venv. The old component stays untouched in `components/canopen`.

Branch: `feature/canopennode` (already created from `main`).

**Read first:** [canopennode.md](canopennode.md), sections "Current state (handoff)" and "CANopenNode facts the
code depends on". They cover the state of the fork, the library converter, the init order, the `CO_config_t`
requirements, the OD structs and the callback signatures this step uses. Old code to borrow patterns from:
`components/canopen/__init__.py` (schema, `to_code`, triggers), `components/canopen/canopen.cpp` (`set_canbus`,
`on_frame`, `setup_heartbeat_client`).

Out of scope for this step: SDO server (`CNT_SDO_SRV = 0`), SDO client, PDOs (no `CO_CANopenInitPDO`), entities,
OTA, logs, persistence.

## New tree: `components_next/canopen/`
(`test/stm32l4_canopen.yaml` gets `external_components: - source: ../components_next`)

### `__init__.py`
- `DEPENDENCIES = ["canbus"]`, `MULTI_CONF = True`, namespace `canopen`, class `CanopenComponent`.
- Schema: `id`, `canbus_id` (use_id `CanbusComponent`), `node_id` (1..127),
  `heartbeat_interval` (default 5000ms, same name as the old component),
  `heartbeat_clients: [{node_id, timeout}]` (same as the old `HB_CLIENT_SCHEMA`),
  `on_hb_consumer_event` (trigger with `uint8 node_id`, fired on timeout, same as the old component),
  `sdo_block_transfer_size` (1..127, default 63, accepted now, becomes `-DCO_CONFIG_SDO_SRV_BLKSIZE`),
  `canopennode_path` (default: `CANopenNode` next to this repo,
  `Path(__file__).resolve().parents[3] / "CANopenNode"`; must be absolute because the zephyr converter rejects
  relative `file://`).
- `to_code`: `cg.add_library("CANopenNode", None, f"file://{path}")`, `cg.add_build_flag("-DCO_CONFIG_SDO_SRV_BLKSIZE=N")`,
  `new_Pvariable(id, node_id)`, `set_canbus`, `set_heartbeat_interval`, `add_heartbeat_consumer(node_id, timeout_ms)`
  per client, trigger via `automation.build_automation` (same pattern as `components/canopen/__init__.py` to_code).

### `od.h` / `od.cpp`: runtime OD builder for CANopenNode
- `ObjectDictionary` keeps entries until `finalize()`, then builds a sorted `OD_entry_t` list plus a blank
  terminator, and an `OD_t {size, list}`. Pointers stay stable after that.
- `add_var(index, attr, void *data, len)` → `ODT_VAR` + `OD_obj_var_t`.
- `add_array(index, attr, uint8_t *count, void *data, elem_len, n)` → `ODT_ARR` + `OD_obj_array_t`.
- `find(index)` → `OD_find()`. Storage for the values is owned by the component.
- Attributes: `ODA_SDO_RW`, plus `ODA_MB` for values of 2 bytes or more. Array sub0 uses `ODA_SDO_R`.
  `subEntriesCount` is 1 for a VAR and n + 1 for an ARRAY. `extension = NULL` (init installs its own on
  0x1014/0x1016/0x1017).
- Generic so later steps (PDO records, 0x2000+ entities) can extend it; `add_record()` comes later.

### `driver_can.cpp`: CANopenNode driver functions (declared `extern "C"` in `301/CO_driver.h`)
- `CANptr` = `CanopenComponent *`, so no globals.
- `CO_CANsetConfigurationMode`, `CO_CANsetNormalMode` (sets `CANnormal`), `CO_CANmodule_init` (fills the struct,
  clears the rx/tx arrays), `CO_CANmodule_disable`, `CO_CANrxBufferInit` (11-bit ident | 0x800 if RTR, mask
  `| 0x800`, object, callback), `CO_CANtxBufferInit` (returns `&txArray[index]`), `CO_CANclearPendingSyncPDOs`.
- `CO_CANsend`:
  - if `bufferFull` is already set → `CO_ERROR_TX_OVERFLOW`;
  - otherwise `canbus->send_data(ident & 0x7FF, false, rtr, data, DLC)`;
  - on `ERROR_ALLTXBUSY`/failure → `bufferFull = true`, `CANtxCount++` (this is the software TX retry queue).
- `CO_CANmodule_process`: retry tx buffers that have `bufferFull` set (`CO_process` calls it, `CANopen.c:1318`).
- `#include "CANopen.h"` from C++ is fine: the CANopenNode headers have `extern "C"` guards.
- RX: `CanopenComponent::on_frame(can_id, extended, rtr, data)`:
  - ignore extended IDs;
  - build `CO_CANrxMsg_t`;
  - call every `rxArray[i]` where `((ident ^ rx.ident) & rx.mask) == 0`;
  - then call `process()` right away, like the old `CONodeProcess` in `on_frame`.

### `canopen.h` / `canopen.cpp`: `CanopenComponent : Component`
- Members: `CO_t *co_`, `CO_config_t config_` (must outlive `co_`), `ObjectDictionary od_`, OD storage
  (`uint32_t dev_type_ = 0`, `uint8_t err_reg_`, `uint32_t cob_emcy_ = 0x80`, `uint16_t hb_producer_ms_`,
  `uint8_t hb_cons_count_`, `std::vector<uint32_t> hb_cons_` with value `(node_id << 16) | timeout_ms`),
  `uint32_t last_us_`, `HbConsumerEventTrigger *on_hb_cons_event_`.
- `setup()` (priority `PROCESSOR`, as before):
  1. Build the OD: 0x1000 u32, 0x1001 u8 (EM), 0x1014 u32 (EMCY COB-ID; CANopenNode adds the node id to 0x80),
     0x1017 u16, 0x1016 array u32 (only if there are clients). Then `finalize()`.
  2. `config_`:
     - `CNT_NMT=1`, `CNT_EM=1`, `CNT_ARR_1003=8` (EM fifo needs ≥ 2), `CNT_HB_CONS = clients ? 1 : 0`,
       `CNT_ARR_1016 = n`, everything else 0;
     - `ENTRY_H1001/1014/1016/1017` = `od_.find(...)`.
  3. `CO_new(&config_, &heap)` → `CO_CANinit(co_, this, 0)` → `CO_CANopenInit(co_, NULL, NULL, od_.od(), NULL,
     CO_NMT_STARTUP_TO_OPERATIONAL, 500 /*first HB*/, 1000, 0, false, node_id, &errInfo)` → `CO_CANsetNormalMode`.
     Log each error with `errInfo` (the OD index that failed).
  4. HB consumer callbacks per index (`CO_CONFIG_HB_CONS_CALLBACK_MULTI`): `initCallbackHeartbeatStarted`,
     `initCallbackNmtChanged`, `initCallbackTimeout` (fires the trigger), `initCallbackRemoteReset`, each logging
     `node 0x%02x ...`. Use `CO_NMT_initCallbackChanged` to log our own NMT state (static function, no context).
- `loop()` → `process()`:
  - `dt = micros() - last_us_` (uint32 wrap is fine);
  - `CO_process(co_, false, dt, nullptr)`;
  - on `CO_RESET_APP` → `App.safe_reboot()`; on `CO_RESET_COMM` → log only (re-init comes later).
- `set_canbus()` registers `add_callback` with the `(can_id, extended, rtr, data)` lambda. The send/callback API
  is the fork's `canbus.h` (`send_data(id, ext, rtr, const uint8_t *, len)`).

## `test/stm32l4_canopen.yaml`
What it has now:
- `esphome: name: stm32l4-blinking-led`, `zephyr: variant STM32L4, board nucleo_l476rg`, MCUboot overlays;
- `canbus: platform zephyr_can, id can_bus, 125kbps, rx_buffer_size 64, tx_buffer_size 32, PA11/PA12`, with an
  `on_frame` on 0x700/0xf80 that blinks the LED;
- `canopen: {id: can_gate, canbus_id: can_bus, node_id: 0x7f, sdo_block_transfer_size: 63}`.

Changes:
- Point `external_components` to `../components_next`.
- Add `heartbeat_interval: 1s` and an example `heartbeat_clients: - node_id: 0x01, timeout: 3s` (to adjust to
  a real node on the bus).
- Drop `logger: logs: canopen-hb` if it's no longer needed. The rest stays (the 0x700 `on_frame` LED blink is a
  nice independent check).

## Verification
1. Host check of the driver/port: compile `driver_can.cpp`/`od.cpp` is covered by the real build. The library
   already host-compiled cleanly with the port config.
2. Config: `../esphomeplatformzephyr/venv/bin/esphome config test/stm32l4_canopen.yaml`.
3. Build: `../esphomeplatformzephyr/venv/bin/esphome compile test/stm32l4_canopen.yaml`. Then check in
   `test/.esphome/build/stm32l4-blinking-led/.west_build/zephyr/compile_commands.json` that `CO_NMT_Heartbeat.c` and
   `canopen.cpp` get the same `-DCO_CONFIG_SDO_SRV_BLKSIZE` (PR #196 path) and include `port/esphome`.
4. On hardware (if the nucleo is attached; `esphome upload` / `logs` uses openocd + UART):
   - boot log shows `CO_CANopenInit` OK and NMT operational;
   - a heartbeat goes out on 0x77F every `heartbeat_interval` (seen by another node / can2mqtt / candump);
   - with a configured client that's alive: "heartbeat started" and NMT state logged;
   - unplug it → timeout log + `on_hb_consumer_event`.

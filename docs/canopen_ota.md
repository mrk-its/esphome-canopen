# CAN OTA in the hub (CiA 302-3 program download)

Plan (2026-11-01). Hub side implemented 2026-11-02 on branch `canopen-full` of `~/repos/esphome`:
`esphome/components/canopen/ota/`. can2mqtt side on its
`dev` (0.2.13-b4). Tested over CAN on the nucleo. Not yet: removing the mapper's OTA. Move OTA out of `canopen_entities` into the `canopen` hub, as
`ota: - platform: canopen`, on the standard program download objects of CiA 302-3 (`0x1F50` / `0x1F51` / `0x1F56` /
`0x1F57`) instead of our manufacturer-specific `0x3000`. Context: [canopennode.md](canopennode.md) (the port, the
upstream split), the current protocol in CLAUDE.md ("CAN OTA flow") and [ota.md](ota.md) (user docs).

## Why

- `0x2000`-`0x5FFF` is the manufacturer-specific area: by convention it belongs to the device application, here
  `canopen_entities` (entities at `0x2000+`, OTA `0x3000`, layout hash `0x3002`, logs `0x3100`, remote state
  slots `0x4000+`). The hub is a generic CiA 301 node that other components (profiles, user objects) add their own
  objects to, in the same area: a fixed `0x3000` in the hub would be a reservation every one of them has to avoid.
- OD collisions are only found at boot (`od.finalize()`: "OD 0x%04x added twice", setup fails), not in config
  validation.
- OTA in the mapper ties firmware updates to the mapper. In the hub, every ESPHome CANopen node can be updated over
  CAN (e.g. a node that only drives a CiA 402 drive).
- A standard upload path is easier to accept upstream than a private OD layout, and any CiA 302-3 capable tool can
  use it.
- Side effect: the mapper loses OTA, which removes the odd `ota: - platform: canopen_entities` (and the same
  oddity under a new mapper name, see the naming discussion: `canopen_entities` preferred).

## CiA 302-3 program download

Reference implementation: Zephyr's CANopenNode module (CANopenNode v1.3),
`zephyr/modules/canopennode/canopen_program.c` (e.g. `~/.cache/esphome/sdk-zephyr/frameworks/sdk-zephyr-v4.4.2/`).
CANopenNode v4 itself (what the hub uses) has no program download objects (`doc/deviceSupport.md` lists "Program
Download" only for the Zephyr port).

**Not checked against CiA 302-3** (v4.1.0; needs CiA membership or purchase, not found publicly). Where the details come
from (2026-11-02):

| Detail | Source | Status |
|---|---|---|
| `0x1F50` / `0x1F51` arrays, sub 0 count, sub 1 program 1, `0x1F51` U8 | DSP 302 v3.2.1 (2003 draft, "not recommended for implementation"), Paulus bootloader manual | supported |
| Data format is the manufacturer's | same draft: "has to be specified by the manufacturer (e.g. INTEL-HEX format or binary format)" | supported |
| `0` stop, `1` start; failed download aborts `0606 0000h`; impossible action aborts `0800 0024h` | same draft | supported, but old |
| `3` clear | Paulus, Zephyr, CANopenNodeZephyr | implementations only |
| `0x1F56` is a CRC-32 | Zephyr, CANopenNodeZephyr; Paulus says "CRC sum" | implementations only |
| `0x1F57` bits and error codes | Zephyr only | weakest |
| Clear only from stop | Zephyr only | probably Zephyr's choice |
| Writes only in PRE-OPERATIONAL | Zephyr only | not adopted, see "Design" |

The draft defines `2` as "reset program", Zephyr rejects it, so do we. Check all of this when a copy of v4.1.0 is at hand.
CiA 710 (a newer draft on CANopen firmware updates) may matter too; not read.

| Object | Type | Zephyr's implementation |
|---|---|---|
| `0x1F50:1` Program data | DOMAIN, write-only | the image; only accepted in state CLEAR (else abort "data cannot be transferred because of the present device state", flash status "not cleared"); first segment `flash_img_init()`, every segment `flash_img_buffered_write()`; last segment: checks the size the client indicated (if any), status -> STOP |
| `0x1F51:1` Program control | U8, rw | writes only in NMT **PRE-OPERATIONAL**; read = status. `0` stop, `1` start, `2` reset (unsupported, abort "invalid value"), `3` clear (only from STOP: erases slot 1 unless erasing progressively), `0x80` Zephyr-specific confirm |
| `0x1F56:1` Program software identification | U32, read-only | CRC32 (IEEE) of the image that runs or will run at the next "start": slot 1 if a download completed, else slot 0, over the size in the MCUboot header |
| `0x1F57:1` Flash status identification | U32, read-only | bit 0 in progress; bits 1-7 error: 0 none, 1 no valid program, 2 data format unknown, 3 data format error, 4 flash not cleared, 5 flash write error, 6 general address error, 7 flash secured, 63 unspecified |

Zephyr's sequence (host): NMT pre-operational -> `0x1F51` = 0 (stop) -> `0x1F51` = 3 (clear) -> download `0x1F50` (block
transfer) -> read `0x1F57` (no error) and `0x1F56` (compare with the CRC32 of the image) -> `0x1F51` = 1 (start: Zephyr
calls `boot_request_upgrade(BOOT_UPGRADE_TEST)` and resets the application). After the reboot an unconfirmed MCUboot
image reads status `0x80`; writing `0x80` confirms it (`boot_write_img_confirmed()`), then status START.

## Design

Hub, `ota: - platform: canopen` (`canopen_id`, `reboot`), objects in the hub's own area (`0x1000`-`0x1FFF`). All of it
is in `canopen/ota/`: without the platform nothing is copied or compiled, and block transfer isn't enabled by it (checked
on ESP8266: no `ota`, `md5`, `safe_mode` components, no symbols; the platform adds 9.4 KB of code there).
`ProgramDownload` (`program_download.*`) has the OTA backend behind virtual methods, so the unit tests can use a fake one:
the host backend's `end()` renames the upload over the running executable. `CANopenOTAComponent` (`canopen_ota.h`) connects
it to the platform's backend.

The image is the platform's plain OTA file, no wrapper: `firmware.ota.bin`, `firmware.bin`, `zephyr.signed.bin`.

Host sequence: stop -> clear -> block download of `0x1F50:1` -> check `0x1F57` and `0x1F56` -> start. On failure: stop.
The application keeps running throughout, "stopped" describes only the download.

- **`0x1F51` = 3 (clear):** `ota->begin(0)`. Size 0 = unknown, because a CANopenNode v4 domain download doesn't pass
  the client's indicated size to the handler (the stream size is 0 until the last chunk, then the total; see
  "OTA ported" in canopennode.md). The current OTA gets it from `0x3000:2` instead. All upstream backends accept 0
  (libretiny `UPDATE_SIZE_UNKNOWN`, rp2 the free space, esp8266 the free sketch space, esp-idf lazy erase instead of
  `esp_ota_begin()`'s full erase); the fork's Zephyr backend skips its size check for 0. None blocks the reply to this
  write. Only from stop (else abort `0800 0024h`); after a finished image it drops that image first.
- **`0x1F50:1`:** `OdObject` write handler, `ota->write()` per chunk (as `FwImageWrite` today); at the last chunk the
  image is complete, status STOP. A new transfer after part of an image is refused (it needs stop and clear). Failures
  abort `0606 0000h`.
- **`0x1F51` = 1 (start):** `ota->end()`, then the reboot, both deferred to `loop()` as today: the SDO reply to this
  write must go out first (on some platforms `backend->end()` reboots right away). Simpler than today: the transfer
  of the image is already finished, so only the reply to one expedited write has to leave. `end()` failing ->
  flash status error, no reboot (but the SDO write is already acknowledged: report it through `0x1F57`).
- **`0x1F51` = 0 (stop):** abort the backend (`ota->abort()`) during a download or with a finished image, so a host that
  found a CRC mismatch can drop it. The error stays in `0x1F57` until the next clear. `0x1F51` = 1 without an image only
  goes back to running. Any other value: abort "invalid value".
- **`0x1F56:1`:** CRC32 (IEEE, same as Python's `zlib.crc32`) of the image passed to the backend (computed while
  writing, no flash read-back, which differs per platform); 0 until a download has finished. `helpers.h` has no CRC32, so
  the hub has a small bitwise one (no table: the data arrives at CAN speed). Not for the running image.
- **`0x1F57:1`:** bit 0 in progress; bits 1-7 Zephyr's error codes (2 format unknown, 4 not cleared, 5 write error, 63
  unspecified); bits 16-23 the backend's `OTAResponseTypes`, for diagnostics.
- **No NMT state required** (decided 2026-11-02; Zephyr requires PRE-OPERATIONAL, nothing we could read requires it).
  The sequence and recovery are simpler: a host that dies mid-upload doesn't leave the node with PDOs off. TPDOs keep
  competing with the transfer, and an upload can start at any time in operation.
- **`0x80` confirm (Zephyr extension):** not implemented, `safe_mode` confirms the image.
- **OTA state listeners** (`on_begin` / `on_progress` / `on_end` / `on_error`), `reboot: false`, the Zephyr
  `safe_mode: disabled` warning and the MCUboot image check on the first chunk (`3d b8 f3 96`) come from the mapper. The
  node doesn't know the image size, so `on_progress` gets 0% (Home Assistant's progress comes from can2mqtt). Only
  `USE_OTA_MCUBOOT` builds check the magic.
- Needs SDO block transfer for usable speed (upstream PR 6, `sdo_block_size`), as today.

### Integrity (replaces the MD5 in `0x3000:3`)

Decided (2026-11-02): no MD5 in the hub OTA; CRC32 instead, and MD5 becomes optional in the Zephyr backend.

CiA 302-3 has no slot for an MD5. Today the host writes the MD5 of the uncompressed image, and the backend checks it
in `end()`. The checks of the new path:

| Check | Covers | Checked by | On failure |
|---|---|---|---|
| SDO block CRC-16 (`CO_CONFIG_CRC16`, on in the hub) | one block transfer | node (`CO_SDOserver.c`) | the transfer aborts |
| CRC32 in `0x1F56:1` | the image, from the host's file to what went to the backend | host, before "start" | no "start"; the old firmware keeps running |
| MCUboot (Zephyr only) | the image as stored in slot 1: hash and signature (ECDSA P256 in `stm32l4-biuro-switch`) | MCUboot, after the reboot | slot scrambled, the old firmware keeps running |

- The node can't judge the CRC32 itself: it isn't told the expected value, so **the host must check it before "start"**.
  A host that skips the check gets a bad image started. On Zephyr MCUboot still rejects it (at the cost of a reboot; the
  host has to notice the node came back on the old firmware). As far as known, ESP-IDF also validates the image at
  `esp_ota_end` (not checked); ESP8266 has no such check, there the host's CRC32 check is the main protection.
- Nothing is lost against MD5 in what the node hashes: the Zephyr backend also hashes the data as it goes to the flash
  write, without a read-back. Neither detects a bad flash write; MCUboot does.
- MD5 in the Zephyr backend protected against accidental corruption only, and so does CRC32. Neither is
  authentication. The MCUboot signature isn't either while the build uses the public sample key
  `root-ec-p256.pem`.

Rejected: MD5 in a manufacturer object or `0x1F50:2` (non-standard again); MD5 in a header in front of the image in
`0x1F50:1` (a generic tool couldn't upload a plain image); the node computing the MD5 itself for `set_update_md5()`
(the check can never fail, the image is hashed twice).

### Zephyr backend: MD5 optional (fork change)

In `esphomeplatformzephyr`, `ota_backend_zephyr.*`. The other backends already treat the MD5 check as optional
(esp-idf, esp8266, host, rp2, libretiny: checked); the Zephyr one requires it (`end()` fails with "No valid MD5
received"). Make it work like esp-idf, esp8266 and host:

- Keep the `md5_` member, with `init()` and `add()` running from `begin()` as they do now.
- In `end()`, compare only if an MD5 was set. **An invalid hash still fails the update** (today's behaviour, see the
  comment there), so `set_update_md5()` has to remember "invalid" apart from "not given".

This works with any call order, which matters because the backend contract in `ota_backend.h` has no ordering rule:
`ota_esphome.cpp` sets the MD5 before the data, `ota_http_request.cpp` and the mapper's `ota/ota.cpp` after it. No
change in the callers or in `ota/__init__.py`.

Cost: MD5 stays in the firmware (about 2.5 KB in `stm32l4-biuro-switch`, of 122 KB of code), and a node on the hub OTA
hashes each image with both MD5 and CRC32. Negligible CPU at CAN speeds.

Rejected: creating the hasher only in `set_update_md5()`, so that the linker drops MD5 when nothing calls it (saves
about 2.4 KB). The hash would miss everything written before that call, so a caller that sets the MD5 after the data
would get a good image rejected. If flash gets tight, add an explicit build option to remove MD5 instead.

### Compression (replaces flag bit 0 of `0x3000:5`)

zlib (miniz) inflate, ~43 KB of heap, off on stm32 / zephyr today; the host compresses only if the node says it
expects compressed data. Options:
1. Drop it in the hub OTA (only ESP32 / ESP8266 benefit). Measure first what it gains over CAN.
2. A second program number: `0x1F50:2` = the same program, zlib-compressed. Self-describing (the host reads
   `0x1F50:0`, the highest sub-index), no manufacturer object, but stretches what program numbers mean.
3. Detect the zlib header on the first chunk; the host still needs a capability bit somewhere (manufacturer object).

Decided for now (2026-11-02): 1, no compression in the hub OTA; measure over CAN before adding any. The backend contract's
`supports_compression()` is worth a look then: true on ESP8266 and rp2 (recent core), where a gzip image is just a format
the backend takes, with no miniz or new object; the host could try it and fall back on "data format unknown". The CRC32
then covers the gzip file. It doesn't help ESP32.

## can2mqtt

`src/can2mqtt/program_download.py`, tests against a fake node (`tests/test_firmware_upload.py`, `fake_ota_node.py`).

- Upload: stop, clear, block download of `0x1F50:1`, check `0x1F57` and `0x1F56` (`zlib.crc32` of the image; **mandatory**,
  see "Integrity"), start, wait up to 60 s for boot-up (none: read `0x1F57`; an error is a failure, otherwise a warning, e.g.
  `reboot: false`). On failure: stop, so the backend isn't left active. Progress as before (`progress_cb`).
- Path per node at registration: `0x1F51:1` readable means 302-3 (also on a firmware with both), otherwise `0x3000`,
  otherwise no update entity. `0x3000` stays for `canopen_entities` until its OTA goes and for nodes on the old component.
- Never compressed on this path, whatever the command topic says.
- The block CRC-16 is in use: `request_crc_support` defaults to on in can2mqtt's canopen-asyncio, and the tests see it.
- The objects are in `eds/esphome.eds` by hand (also in `header.eds.tpl`): `create.sh` is stale, it drops `0x3100`.
- The mapper's identity check (`ESPHOME_VENDOR_ID` / `ESPHOME_PRODUCT_CODE`) stays, so a node with the hub OTA but no
  `canopen_entities` isn't registered, and can't be updated from can2mqtt yet.

## Upstream

PR 7 of the new split (canopennode.md, "Upstream PRs"): `[canopen] Add OTA on the CiA 302-3 program download objects`,
branch `canopen-pr7-ota`, stacked on PR 6. It uses the OD handler API (PR 4) and needs SDO block transfer (PR 6), so it
goes out after those; only PR 1 is cut so far. Size: 388 lines + 277 of tests by `scripts/pr_split_lines.py`, under the
~1000-line limit, and all in new files: `ota/` and `test-ota.*`, nothing in the hub's shared files.

**Contents:** `esphome/components/canopen/ota/` (`__init__.py`, `canopen_ota.h`, `program_download.*`),
`tests/components/canopen/ota/`, `test-ota.esp32-idf.yaml`, `test-ota.esp8266-ard.yaml`, the two tests and two configs in
`tests/component_tests/canopen/`.

**Left out of the PR: the Zephyr parts** (decided 2026-11-02), because upstream has no Zephyr backend, so they would be
dead code there: the `SWAP_METHOD_SCHEMA` import, `_mcuboot()`, `_is_zephyr()` and the `safe_mode` warning in `__init__.py`,
and the `USE_OTA_MCUBOOT` magic check in `program_download.cpp`. They stay on `canopen-full`, which then differs from the
last PR branch by exactly these lines (check it with the "must equal `canopen-full`" diff), and go upstream with the Zephyr
platform or a follow-up. MCUboot rejects a bad image anyway; the magic check only fails it earlier.

**Before it goes out:**
- [ ] Check the objects against the CiA 302-3 v4.1.0 text (table in "CiA 302-3 program download"). If no copy can be had,
  say in the PR what they're based on, not that they follow the standard.
- [ ] A real CAN upload on an upstream platform. So far only the nucleo on the Zephyr fork, which upstream doesn't have:
  ESP32 (ESP-IDF) and ESP8266 at least; `begin(0)` is read and compiled there, not run.
- [ ] An interrupted upload over CAN (host gone mid-transfer): the application keeps running, the next stop + clear works.
- [ ] Clang-tidy (`esp8266-arduino-tidy`, `esp32-idf-tidy`, `bk72xx-tidy`, `nrf52-tidy`). Not run yet. It may need `defines.h`
  / `platformio.ini` additions as in "Cutting the branches", step 2. nrf52 has no backend with this contract: check the
  platform's validation there (`test-ota` only covers ESP32 and ESP8266).
- [ ] `test_build_components.py` on `esp32-ard` and `rp2040-ard` too (add `test-ota` files, or only say which ran). Bk72xx,
  rtl87xx: no canbus platform in the tests today.
- [ ] Docs: an OTA section on the `canopen` page in esphome-docs, branch `canopen-full`, then cut with the PR: the option,
  the host sequence and that the host must check the CRC, the image file per platform.
- [ ] `canopen-pr7-ota` on its own: `cpp_unit_test.py canopen`, the component tests, pre-commit.

**PR description points:**
- Why standard objects: a private OD index in the hub would be a reservation every other user of the manufacturer area has
  to avoid, and any CiA 302-3 tool can use these.
- Not compiled in unless requested (the ESP8266 comparison: 0 symbols without, +9.7 KB of code with).
- The host is responsible for the CRC check; no MD5, no compression, and no NMT state required, each a deliberate choice.
- The block size cap of PR 6 depends on the CANopenNode fork's patch, as PR 6 already says.
- Client: can2mqtt's `program_download.py` as the reference.

After it: can2mqtt keeps both paths, the mapper's OTA goes (plan step 7).

## Testing done (2026-11-02)

- `script/cpp_unit_test.py canopen` (`tests/components/canopen/ota/`: the state machine on a fake backend),
  `tests/component_tests/canopen` (nothing without the platform), `test_build_components.py -c canopen` on esp32-idf and
  esp8266-ard (with `test-ota.*` too).
- Nucleo (Zephyr, the backend of esphomeplatformzephyr PR 231, the hub from the fork's working tree): a test firmware
  driving the objects through the OD with its own signed image from slot 0. Refusals, CRC, stop, swap, confirmation; a
  corrupted image gets a different CRC, and if started anyway MCUboot rejects it.
- Over CAN, 2026-11-02: can2mqtt 0.2.13-b4 updated the nucleo (`sdo_block_size: 60`): 122,455 B in 46 s, 2.62 KB/s, no gap
  over 1 s, CRC-32 as the file's, MCUboot swap about 6 s, `safe_mode` confirmed the image. The first real upload, Zephyr only.
  Not yet: an interrupted upload, ESP32, ESP8266, a node moving from `0x3000`.

## Open questions

- Check the objects against CiA 302-3 v4.1.0 (see the table above).
- Migration of existing nodes: an update from the mapper's `0x3000` OTA to a firmware with the hub OTA goes through
  the old path once; can2mqtt picks the path per node, so nothing special is needed. Check on the nucleo.

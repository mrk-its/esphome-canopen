# CAN OTA in the hub (CiA 302-3 program download)

Plan (2026-11-01, nothing implemented): move OTA out of `canopen_entity_mapper` into the `canopen` hub, as
`ota: - platform: canopen`, on the standard program download objects of CiA 302-3 (`0x1F50` / `0x1F51` / `0x1F56` /
`0x1F57`) instead of our manufacturer-specific `0x3000`. Context: [canopennode.md](canopennode.md) (the port, the
upstream split), the current protocol in CLAUDE.md ("CAN OTA flow") and [ota.md](ota.md) (user docs).

## Why

- `0x2000`-`0x5FFF` is the manufacturer-specific area: by convention it belongs to the device application, here
  `canopen_entity_mapper` (entities at `0x2000+`, OTA `0x3000`, layout hash `0x3002`, logs `0x3100`, remote state
  slots `0x4000+`). The hub is a generic CiA 301 node that other components (profiles, user objects) add their own
  objects to, in the same area: a fixed `0x3000` in the hub would be a reservation every one of them has to avoid.
- OD collisions are only found at boot (`od.finalize()`: "OD 0x%04x added twice", setup fails), not in config
  validation.
- OTA in the mapper ties firmware updates to the mapper. In the hub, every ESPHome CANopen node can be updated over
  CAN (e.g. a node that only drives a CiA 402 drive).
- A standard upload path is easier to accept upstream than a private OD layout, and any CiA 302-3 capable tool can
  use it.
- Side effect: the mapper loses OTA, which removes the odd `ota: - platform: canopen_entity_mapper` (and the same
  oddity under a new mapper name, see the naming discussion: `canopen_entities` preferred).

## CiA 302-3 program download

Reference implementation checked: Zephyr's CANopenNode module (CANopenNode v1.3),
`zephyr/modules/canopennode/canopen_program.c` (e.g. `~/.cache/esphome/sdk-zephyr/frameworks/sdk-zephyr-v4.4.2/`).
CANopenNode v4 itself (what the hub uses) has no program download objects (`doc/deviceSupport.md` lists "Program
Download" only for the Zephyr port). Check the details against the CiA 302-3 text before implementing.

| Object | Type | Zephyr's implementation |
|---|---|---|
| `0x1F50:1` Program data | DOMAIN, write-only | the image; only accepted in state CLEAR (else abort "data cannot be transferred because of the present device state", flash status "not cleared"); first segment `flash_img_init()`, every segment `flash_img_buffered_write()`; last segment: checks the size the client indicated (if any), status -> STOP |
| `0x1F51:1` Program control | U8, rw | writes only in NMT **PRE-OPERATIONAL**; read = status. `0` stop, `1` start, `2` reset (unsupported, abort "invalid value"), `3` clear (only from STOP: erases slot 1 unless erasing progressively), `0x80` Zephyr-specific confirm |
| `0x1F56:1` Program software identification | U32, read-only | CRC32 (IEEE) of the image that runs or will run at the next "start": slot 1 if a download completed, else slot 0, over the size in the MCUboot header |
| `0x1F57:1` Flash status identification | U32, read-only | bit 0 in progress; bits 1-7 error: 0 none, 1 no valid program, 2 data format unknown, 3 data format error, 4 flash not cleared, 5 flash write error, 6 general address error, 7 flash secured, 63 unspecified |

Sequence (host): NMT pre-operational -> `0x1F51` = 0 (stop) -> `0x1F51` = 3 (clear) -> download `0x1F50` (block
transfer) -> read `0x1F57` (no error) and `0x1F56` (compare with the CRC32 of the image) -> `0x1F51` = 1 (start: Zephyr
calls `boot_request_upgrade(BOOT_UPGRADE_TEST)` and resets the application). After the reboot an unconfirmed MCUboot
image reads status `0x80`; writing `0x80` confirms it (`boot_write_img_confirmed()`), then status START.

## Design

Hub, `ota: - platform: canopen` (`canopen_id`), objects in the hub's own area (`0x1000`-`0x1FFF`):

- **`0x1F51` = 3 (clear):** `ota->begin(0)`. Size 0 = unknown, because a CANopenNode v4 domain download doesn't pass
  the client's indicated size to the handler (the stream size is 0 until the last chunk, then the total; see
  "OTA ported" in canopennode.md). The current OTA gets it from `0x3000:2` instead. All upstream backends accept 0
  (libretiny `UPDATE_SIZE_UNKNOWN`, rp2 the free space, esp8266 the free sketch space, esp-idf sequential writes);
  the fork's Zephyr backend skips its size check for 0. The erase happens in `begin()` / lazily, as today.
- **`0x1F50:1`:** `OdObject` write handler, `ota->write()` per chunk (as `FwImageWrite` today); at the last chunk the
  image is complete, status STOP. Optional zlib inflate in between (see below).
- **`0x1F51` = 1 (start):** `ota->end()`, then the reboot, both deferred to `loop()` as today: the SDO reply to this
  write must go out first (on some platforms `backend->end()` reboots right away). Simpler than today: the transfer
  of the image is already finished, so only the reply to one expedited write has to leave. `end()` failing ->
  flash status error, no reboot (but the SDO write is already acknowledged: report it through `0x1F57`).
- **`0x1F51` = 0 (stop) during a download:** abort the backend (`ota->abort()`).
- **`0x1F56:1`:** CRC32 of the received image (computed while writing, no flash read-back, which differs per
  platform); 0 before any download. Optionally the running image's CRC32, stored in preferences at "start" (not
  needed for the upload; open).
- **`0x1F57:1`:** in progress / error code from the backend's `OTAResponseTypes`.
- **PRE-OPERATIONAL only**, as Zephyr: no PDOs during the upload (less bus load during block transfer, no TPDO frames
  in the RX queue of the host), and an upload can't start by accident. can2mqtt then has to send NMT pre-operational
  first (its `firmware_upload()` already has it as a TODO) and NMT start if the upload fails.
- **`0x80` confirm (Zephyr extension):** on Zephyr, ESPHome's `safe_mode` confirms the image 60 s after boot. Support
  `0x80` too (status `0x80` while unconfirmed, writing it confirms early)? Lets the host confirm right after the
  upload and see whether the new image runs; not needed for can2mqtt today. Open.
- **OTA state listeners** (`on_begin` / `on_progress` / `on_end` / `on_error`), `reboot: false`, the Zephyr
  `safe_mode: disabled` warning and the MCUboot image check on the first chunk (`3d b8 f3 96`) move from the mapper
  as they are.
- Needs SDO block transfer for usable speed (upstream PR 6, `sdo_block_size`), as today.

### Integrity (replaces the MD5 in `0x3000:3`)

CiA 302-3 has no slot for an MD5. Today the host writes the MD5 of the uncompressed image, and the backend checks it
in `end()`.

- Upstream backends (esp-idf, esp8266, rp2, libretiny, host): MD5 optional (`md5_set_`).
- Fork's Zephyr backend (`ota_backend_zephyr.cpp`): **requires** it, `end()` fails with "No valid MD5 received".
- SDO block transfer has a CRC of the whole transfer (CRC-16, if both sides support it), the host checks
  `0x1F56` (CRC32 of what the node received) before "start".

Options:
1. Host check only: `0x1F56` CRC32 (+ the block CRC). The node computes the MD5 of what it received itself and passes
   it to `set_update_md5()` (for the Zephyr backend), or the Zephyr backend accepts no MD5 like the others.
2. Keep an MD5 as a manufacturer extension (e.g. a sub of `0x1F56`'s neighbourhood, or `0x1F50:2`): non-standard
   again.

Preferred: 1, the Zephyr backend made optional like the upstream ones (fork change, not a CANopenNode one).

### Compression (replaces flag bit 0 of `0x3000:5`)

zlib (miniz) inflate, ~43 KB of heap, off on stm32 / zephyr today; the host compresses only if the node says it
expects compressed data. Options:
1. Drop it in the hub OTA (only ESP32 / ESP8266 benefit). Measure first what it gains over CAN.
2. A second program number: `0x1F50:2` = the same program, zlib-compressed. Self-describing (the host reads
   `0x1F50:0`, the highest sub-index), no manufacturer object, but stretches what program numbers mean.
3. Detect the zlib header on the first chunk; the host still needs a capability bit somewhere (manufacturer object).

Open; 1 or 2.

## can2mqtt

- New upload path for 302-3: NMT pre-operational, stop, clear, block download of `0x1F50:1`, check `0x1F57` /
  `0x1F56`, start; NMT start on failure. Progress as today (`progress_cb`).
- Detect which one a node has (SDO read of `0x1F51:1` vs `0x3000`); keep the `0x3000` path for nodes on the old
  component (canopen-stack), which keeps `0x3000` unchanged.
- The mapper's identity check (`ESPHOME_VENDOR_ID` / `ESPHOME_PRODUCT_CODE`) stays; OTA no longer depends on it.

## Upstream

- A later hub PR on top of PR 6 (SDO block transfer), after the six of the new split (canopennode.md, "Upstream PRs"):
  `ota/` subdirectory of `canopen` (platform `ota: - platform: canopen`), miniz only if compression stays.
- Docs: an OTA section on the `canopen` page in esphome-docs (branch `canopen-full`).

## Open questions

- MD5: option 1 (CRC32 in `0x1F56`, Zephyr backend without MD5) or keep an MD5 extension?
- Compression: drop, `0x1F50:2`, or a capability bit?
- `0x80` confirm: implement (and on which platforms: MCUboot only, or ESP-IDF rollback too)?
- PRE-OPERATIONAL only: agreed? It changes can2mqtt's sequence (NMT around the upload).
- `0x1F56` for the running image (preferences) or only for a received one?
- Migration of existing nodes: an update from the mapper's `0x3000` OTA to a firmware with the hub OTA goes through
  the old path once; can2mqtt picks the path per node, so nothing special is needed. Check on the nucleo.

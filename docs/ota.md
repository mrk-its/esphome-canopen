# OTA over CAN
The `canopen` OTA platform allows to flash firmware over CAN bus, so a node needs USB/serial access only for the
first flash. [can2mqtt](https://github.com/mrk-its/can2mqtt) uploads the image and shows the update in Home
Assistant.
```yaml
ota:
  - platform: canopen
```

## Updating a node
1. Build the firmware: `esphome compile node.yaml`. The image is
   `.esphome/build/<node>/.pioenvs/<node>/firmware.bin` (on Zephyr see [below](#zephyr-platform-zephyr-with-mcuboot)).
2. Copy it into a per-node subdirectory of can2mqtt's firmware directory, e.g. `firmware/<node>/firmware.bin`:
   * HA add-on: `/addon_configs/<SLUG>_can2mqtt/<node>/firmware.bin` (SLUG is shown on the add-on page)
   * standalone: `--firmware-dir` (default `./firmware`)
3. can2mqtt logs `new_firmware: ...` and an update entity appears in Home Assistant. Install it there. Progress is
   shown in HA and in the can2mqtt log, and the node reboots when the upload has finished.

If the upload is very slow, see [Troubleshooting](../README.md#troubleshooting): CAN OTA uses SDO block transfer
with `sdo_block_transfer_size` frames per block (see [Configuration](configuration.md)), and every block has to fit
the node's canbus RX queue.

On ESP32 / ESP8266 the node asks for a zlib-compressed image (OD `0x3000:5`), and can2mqtt compresses it on the fly.

## Zephyr (`platform: zephyr`) with MCUboot
On `platform: zephyr` the update is written to MCUboot's secondary slot and swapped in on the next reboot:
* MCUboot is required: set `advanced: bootloader: mcuboot` in the `zephyr:` block (with a partition layout that has `slot0_partition` / `slot1_partition`).
* `swap_method:` (`scratch`, `move` or `offset`) can be set on the `canopen` OTA platform, same as on `platform: esphome`.
* Upload the signed image: `.esphome/build/<node>/.west_build/zephyr/zephyr/zephyr.signed.bin`. Never upload `zephyr.bin` (unsigned); the node rejects it at the start of the transfer.
* The new image boots in test mode. `safe_mode` confirms it after a good boot; if it's never confirmed, MCUboot rolls back to the previous image on the next reboot.
* OTA compression is disabled on Zephyr (and on stm32), so can2mqtt sends the image uncompressed.

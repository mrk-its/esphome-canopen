#pragma once

#include "esphome/components/canopen/canopen.h"
#include "esphome/components/ota/ota_backend.h"
#include "esphome/components/ota/ota_backend_factory.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#ifdef OTA_COMPRESSION
#include "miniz.h"
#endif

namespace esphome::canopen_entities {

// CAN OTA (`ota: - platform: canopen_entities`): firmware download into OD 0x3000 by SDO block transfer, as
// can2mqtt's firmware_upload() does it (OBJECT_DICTIONARY.md):
//   :1 command (ignored), :2 image size, :3 MD5 (16 bytes, of the uncompressed image), :4 image data (domain),
//   :5 flags (bit 0: the node expects zlib-compressed data)
// The first chunk of :4 starts the OTA backend, the last one (the stream size is set) sets the MD5 and finishes it. end() is deferred to
// loop() (~1 s), because on some platforms the backend reboots right away and the SDO transfer must end first.
class CanopenOTAComponent : public ota::OTAComponent, public canopen::OdObject {
 public:
  static constexpr uint16_t OD_INDEX = 0x3000;

  // adds the OD record (from codegen, before the hub's setup)
  void set_canopen(canopen::CANopenComponent *hub);
  void set_reboot(bool reboot) { this->reboot_ = reboot; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  canopen::OdResult od_write(canopen::OdStream stream, const void *buf, size_t count, size_t *done);

 protected:
  enum State : uint8_t { STATE_IDLE, STATE_RECEIVING, STATE_END_WAIT, STATE_REBOOT_WAIT };

  ota::OTAResponseTypes begin_();
  ota::OTAResponseTypes write_(const uint8_t *data, size_t len);
  ota::OTAResponseTypes finish_();
  void fail_(ota::OTAResponseTypes error);
#ifdef OTA_COMPRESSION
  ota::OTAResponseTypes inflate_(bool finish);
#endif

  ota::OTABackendPtr backend_;
  State state_{STATE_IDLE};
  uint32_t state_start_ms_{0};
  bool reboot_{true};

  // OD 0x3000 storage
  uint8_t highest_{5};
  uint32_t command_{0};
  uint32_t size_{0};  // uncompressed image size
  uint8_t md5_[16]{};
  uint32_t flags_{0};

  size_t received_{0};  // bytes of :4 (compressed if compression is on)
  bool check_header_{false};
#ifdef OTA_COMPRESSION
  static constexpr size_t INFLATE_BUF_SIZE = 8 * 1024;
  z_stream stream_{};
  std::unique_ptr<uint8_t[]> inflate_buf_;
#endif
};

}  // namespace esphome::canopen_entities

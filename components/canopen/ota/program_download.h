#pragma once

#include "esphome/components/canopen/canopen.h"
#include "esphome/components/ota/ota_backend.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"

namespace esphome::canopen {

// CRC-32 (IEEE, the same as Python's zlib.crc32), a running value starting at 0. Bitwise: the data arrives at CAN
// speed, so a table isn't worth its flash.
inline uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
  crc = ~crc;
  while (len-- > 0) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

// Firmware download on the program download objects of CiA 302-3 (one program). The OTA backend is behind
// backend_*_(): canopen_ota.h has the component with the platform's backend, the unit tests a fake one.
//   0x1F50:1 program data, the image as the platform's OTA backend takes it. A domain.
//   0x1F51:1 program control. Write: 0 stop, 1 start, 3 clear. Read: 0 stopped, 1 running, 3 cleared.
//   0x1F56:1 program identification: CRC-32 of the received image, 0 until the download has finished.
//   0x1F57:1 flash status: bit 0 in progress, bits 1-7 error, bits 16-23 the backend's OTAResponseTypes.
// The host writes stop, clear, the image, checks 0x1F57 and 0x1F56 (the node isn't told the expected CRC, so the check
// is the host's), then writes start. On failure it writes stop. The application keeps running throughout: "stopped"
// only describes the download, and no NMT state is required.
class ProgramDownload : public ota::OTAComponent, public OdObject {
 public:
  static constexpr uint16_t OD_PROGRAM_DATA = 0x1F50;
  static constexpr uint16_t OD_PROGRAM_CONTROL = 0x1F51;
  static constexpr uint16_t OD_PROGRAM_ID = 0x1F56;
  static constexpr uint16_t OD_FLASH_STATUS = 0x1F57;

  static constexpr uint8_t CONTROL_STOP = 0;
  static constexpr uint8_t CONTROL_START = 1;
  static constexpr uint8_t CONTROL_CLEAR = 3;

  static constexpr uint32_t FLASH_IN_PROGRESS = 1;
  static constexpr uint8_t FLASH_NO_VALID_PROGRAM = 1;
  static constexpr uint8_t FLASH_FORMAT_UNKNOWN = 2;
  static constexpr uint8_t FLASH_NOT_CLEARED = 4;
  static constexpr uint8_t FLASH_WRITE_ERROR = 5;
  static constexpr uint8_t FLASH_UNSPECIFIED = 63;

  // adds the OD objects (from codegen, before the hub's setup)
  void set_canopen(CANopenComponent *hub);
  void set_reboot(bool reboot) { this->reboot_ = reboot; }

  void setup() override {
    // loop() only runs between "start" and the reboot
    this->disable_loop();
  }
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  OdResult od_write(OdStream stream, const void *buf, size_t count, size_t *done);

 protected:
  enum Phase : uint8_t {
    PHASE_IDLE,        // no backend session
    PHASE_CLEARED,     // backend started, no data yet
    PHASE_RECEIVING,   // part of the image received
    PHASE_DOWNLOADED,  // the whole image received, waiting for start or stop
    PHASE_END_WAIT,    // start accepted, backend end() deferred
    PHASE_REBOOT_WAIT,
  };

  virtual ota::OTAResponseTypes backend_begin() = 0;
  virtual ota::OTAResponseTypes backend_write(const uint8_t *data, size_t len) = 0;
  virtual ota::OTAResponseTypes backend_end() = 0;
  virtual void backend_abort() = 0;

  OdResult control_write_(OdStream stream, const void *buf, size_t count, size_t *done);
  OdResult stop_();
  OdResult start_();
  OdResult clear_();
  OdResult data_write_(OdStream stream, const uint8_t *data, size_t count, size_t *done);
  bool finishing_() const { return this->phase_ == PHASE_END_WAIT || this->phase_ == PHASE_REBOOT_WAIT; }
  void set_error_(uint8_t status, ota::OTAResponseTypes response);
  void fail_(ota::OTAResponseTypes error);

  // end() after the reply to "start" has left, the reboot after end()
  uint32_t delay_ms_{1000};
  uint32_t phase_start_ms_{0};
  size_t received_{0};
  uint32_t crc_{0};
  Phase phase_{PHASE_IDLE};
  bool reboot_{true};

  // OD storage
  uint32_t program_id_{0};
  uint32_t flash_status_{0};
  uint8_t programs_{1};
  uint8_t control_{CONTROL_START};
};

}  // namespace esphome::canopen

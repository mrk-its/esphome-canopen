#pragma once

#include "esphome/components/ota/ota_backend_factory.h"
#include "program_download.h"

namespace esphome::canopen {

// `ota: - platform: canopen`: program download (program_download.h) into the platform's OTA backend
class CANopenOTAComponent final : public ProgramDownload {
 public:
  void setup() override {
    this->backend_ = ota::make_ota_backend();
    ProgramDownload::setup();
  }

 protected:
  // 0: the size is unknown, CANopenNode doesn't pass on the size the client indicates. All backends accept it, and
  // none erases here: the reply to the clear isn't held up.
  ota::OTAResponseTypes backend_begin_() override { return this->backend_->begin(0); }
  ota::OTAResponseTypes backend_write_(const uint8_t *data, size_t len) override {
    return this->backend_->write(const_cast<uint8_t *>(data), len);
  }
  ota::OTAResponseTypes backend_end_() override { return this->backend_->end(); }
  void backend_abort_() override { this->backend_->abort(); }

  ota::OTABackendPtr backend_;
};

}  // namespace esphome::canopen

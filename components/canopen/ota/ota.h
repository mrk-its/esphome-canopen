#pragma once
// #include "esphome.h"

// #ifdef USE_OTA

#include "miniz.h"

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "esphome/core/automation.h"

#include "esphome/components/ota/ota_backend.h"

namespace esphome {
namespace canopen {

class OtaFinishedTrigger : public Trigger<> {};

class CanopenOTAComponent : public ota::OTAComponent {
  const char *TAG = "canopen_ota";
  const static uint32_t BUF_SIZE = 8 * 1024;
  OtaFinishedTrigger *ota_finished_trigger;
  z_stream stream;
  uint8_t s_outbuf[BUF_SIZE];
  uint32_t size;
  uint32_t written;
  uint32_t received;
  bool dry_run = false;

  int decompress();

 public:
  std::unique_ptr<esphome::ota::OTABackend> backend;
  void setup() override;
  //   void dump_config() override;
  float get_setup_priority() const override;
  void loop() override;

  esphome::ota::OTAResponseTypes begin(uint32_t size);
  esphome::ota::OTAResponseTypes write(uint8_t *data, size_t len);
  esphome::ota::OTAResponseTypes end(const char *expected_md5);
};
}  // namespace canopen
}  // namespace esphome
// #endif

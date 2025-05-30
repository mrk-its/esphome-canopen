#pragma once
// #include "esphome.h"

// #ifdef USE_OTA

#ifdef OTA_COMPRESSION
#include "miniz.h"
#endif

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
  OtaFinishedTrigger *ota_finished_trigger;

#ifdef OTA_COMPRESSION
  const static uint32_t BUF_SIZE = 8 * 1024;
  z_stream stream;
  uint8_t s_outbuf[BUF_SIZE];
  uint32_t written;
  uint32_t received;
  int decompress();
#endif

  bool dry_run = false;

 public:
  bool disable_ota_reboot = false;
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

#pragma once
// #include "esphome.h"

// #ifdef USE_OTA

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
  OtaFinishedTrigger *ota_finished_trigger;

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

#include "ota.h"
// #ifdef USE_OTA
#include "esphome/components/md5/md5.h"
#include "esphome/components/ota/ota_backend.h"
#include "esphome/components/ota/ota_backend_esp_idf.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/base_automation.h"

#include <cerrno>
#include <cstdio>

namespace esphome {
namespace canopen {

void CanopenOTAComponent::setup() {
#ifdef USE_OTA_STATE_CALLBACK
  ota::register_ota_platform(this);
#endif
  backend = esphome::ota::make_ota_backend();

  ota_finished_trigger = new canopen::OtaFinishedTrigger();
  auto automation_id = new Automation<>(ota_finished_trigger);
  auto delayaction_id = new DelayAction<>();
  delayaction_id->set_component_source("canopen.ota");
  App.register_component(delayaction_id);
  delayaction_id->set_delay(5000);
  auto lambdaaction_id = new LambdaAction<>([=]() -> void {
    ESP_LOGI("main", "ota finished, rebooting");
    App.safe_reboot();
  });
  automation_id->add_actions({delayaction_id, lambdaaction_id});

  // ota_finished_trigger = new OtaFinishedTrigger();

  // auto automation = new Automation<>(reboot_trigger);
  // auto cb = [this]() -> void {
  //   // this->on_frame(can_id, remote_transmission_request, x);
  // };
  // auto delay = new DelayAction<>();
  // auto lambdaaction = new LambdaAction<>(cb);
  // automation->add_actions({delay, lambdaaction});
}

void CanopenOTAComponent::loop() {}

float CanopenOTAComponent::get_setup_priority() const { return setup_priority::LATE; }

esphome::ota::OTAResponseTypes CanopenOTAComponent::begin(uint32_t size) { return backend->begin(size); }
esphome::ota::OTAResponseTypes CanopenOTAComponent::write(uint8_t *data, size_t len) {
  return backend->write(data, len);
}
esphome::ota::OTAResponseTypes CanopenOTAComponent::end(const char *expected_md5) {
  backend->set_update_md5(expected_md5);
  auto ret = backend->end();
  if (!ret) {
    ota_finished_trigger->trigger();
    // esphome::delay(5000);  // NOLINT
    // App.safe_reboot();
  }
  return ret;
}

}  // namespace canopen
}  // namespace esphome
// #endif

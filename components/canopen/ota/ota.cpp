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
  auto delayaction2_id = new DelayAction<>();

  delayaction_id->set_component_source("canopen.ota");
  App.register_component(delayaction_id);
  delayaction_id->set_delay(1000);

  delayaction2_id->set_component_source("canopen.ota");
  App.register_component(delayaction2_id);
  delayaction2_id->set_delay(1000);

  auto ota_end_id = new LambdaAction<>([=]() -> void {
    esphome::ota::OTAResponseTypes ret = esphome::ota::OTAResponseTypes::OTA_RESPONSE_OK;
    if(!dry_run) {
      ret = backend->end();
    }
    if(ret == esphome::ota::OTAResponseTypes::OTA_RESPONSE_OK) {
      ESP_LOGI(TAG, "ota finished successfully, rebooting");
    } else {
      ESP_LOGE(TAG, "ota error: %d", ret);
    }
  });

  auto reboot_action_id = new LambdaAction<>([=]() -> void {
    if(!disable_ota_reboot) {
      ESP_LOGI(TAG, "ota finished, rebooting");
      App.safe_reboot();
    }
  });

  automation_id->add_actions({delayaction_id, ota_end_id, delayaction2_id, reboot_action_id});
#ifdef OTA_COMPRESSION
  this->stream = z_stream{};
#endif
}

void CanopenOTAComponent::loop() {}

float CanopenOTAComponent::get_setup_priority() const { return setup_priority::LATE; }

esphome::ota::OTAResponseTypes CanopenOTAComponent::begin(uint32_t size) {
#ifdef OTA_COMPRESSION
  if(this->stream.state) {
    mz_deflateEnd(&this->stream);
  }

  this->stream.next_out = this->s_outbuf;
  this->stream.avail_out = BUF_SIZE;
  this->stream.avail_in = 0;
  this->written = 0;
  this->received = 0;

  int err = mz_inflateInit(&this->stream);
  if (err) {
    ESP_LOGW(TAG, "cannot initialize decompressor: %d", err);
    return esphome::ota::OTAResponseTypes::OTA_RESPONSE_ERROR_UNKNOWN;
  }
#endif
  return !dry_run ? backend->begin(size) : esphome::ota::OTAResponseTypes::OTA_RESPONSE_OK;
}

#ifdef OTA_COMPRESSION
int CanopenOTAComponent::decompress() {
  auto status = inflate(&this->stream, Z_SYNC_FLUSH);
  if ((status == Z_STREAM_END) || (status == Z_OK && !this->stream.avail_out)) {
    uint32_t n = BUF_SIZE - this->stream.avail_out;
    this->written += n;
    if(n > 0) {
      ESP_LOGI(TAG, "writing %ld bytes to flash, total: %ld", n, this->written);
      auto ret = !dry_run ? backend->write(this->s_outbuf, n) : esphome::ota::OTAResponseTypes::OTA_RESPONSE_OK;
      if (ret != esphome::ota::OTAResponseTypes::OTA_RESPONSE_OK) {
        ESP_LOGW("ota", "write flash error: %d", ret);
        return -10;
      }
    }
    this->stream.next_out = this->s_outbuf;
    this->stream.avail_out = BUF_SIZE;
  }
  return status;
}
#endif

esphome::ota::OTAResponseTypes CanopenOTAComponent::write(uint8_t *data, size_t len) {
#ifdef OTA_COMPRESSION
  ESP_LOGV(
    TAG,
    "offset: %ld, len: %d, data: %02x %02x %02x %02x %02x %02x %02x",
    this->received, len,
    data[0], data[1], data[2], data[3],
    data[3], data[4], data[6]
  );
  this->stream.next_in = data;
  this->stream.avail_in = len;
  this->received += len;

  while (stream.avail_in) {
    int status = this->decompress();
    if (status == Z_STREAM_END) {
      break;
    }
    if (status != Z_OK) {
      ESP_LOGW(TAG, "decompression failed with %d", status);
      return esphome::ota::OTAResponseTypes::OTA_RESPONSE_ERROR_UNKNOWN;
    }
  }
#else
  ESP_LOGI(TAG, "ota write %d bytes", len);
  if(!dry_run) {
    return backend->write(data, len);
  }
#endif
  return esphome::ota::OTAResponseTypes::OTA_RESPONSE_OK;
}

esphome::ota::OTAResponseTypes CanopenOTAComponent::end(const char *expected_md5) {
  esphome::ota::OTAResponseTypes ret;
#ifdef OTA_COMPRESSION
  for (;;) {
    auto status = this->decompress();
    if (status == Z_STREAM_END) {
      ret = esphome::ota::OTAResponseTypes::OTA_RESPONSE_OK;
      if (!dry_run) {
        backend->set_update_md5(expected_md5);
      }
      ota_finished_trigger->trigger();
      // if (!ret) {
      //   ESP_LOGI(TAG, "decompression finished successfully");
      //   if(!dry_run && !disable_ota_reboot) {
      //     ota_finished_trigger->trigger();
      //   }
      // }
      break;
    } else if (status != Z_OK) {
      ESP_LOGW(TAG, "decompression failed with %d", status);
      ret = esphome::ota::OTAResponseTypes::OTA_RESPONSE_ERROR_UNKNOWN;
      break;
    }
  }
  mz_deflateEnd(&this->stream);
#else
  ret = esphome::ota::OTAResponseTypes::OTA_RESPONSE_OK;
  if(!dry_run) {
    backend->set_update_md5(expected_md5);
  }
  // let's finish update asynchronously
  // as on stm32 platform calling backend->end() instantly reboots
  // and canopen block transfer is not finished
  ota_finished_trigger->trigger();
#endif
  return ret;
}

}  // namespace canopen
}  // namespace esphome
// #endif

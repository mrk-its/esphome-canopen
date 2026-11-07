#include "program_download.h"

#include <cinttypes>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome::canopen {

ESPHOME_LOG_TAG(TAG, "canopen.ota");

void ProgramDownload::set_canopen(CANopenComponent *hub) {
  auto &od = hub->get_od();
  // a domain without storage: the handler takes it in chunks
  od.add_record(OD_PROGRAM_DATA, {{0, OD_READ, &this->programs_, 1}, {1, OD_WRITE, nullptr, 0}});
  od.set_object(OD_PROGRAM_DATA, this);
  od.add_record(OD_PROGRAM_CONTROL, {{0, OD_READ, &this->programs_, 1}, {1, OD_READ_WRITE, &this->control_, 1}});
  od.set_object(OD_PROGRAM_CONTROL, this);
  od.add_record(OD_PROGRAM_ID, {{0, OD_READ, &this->programs_, 1}, {1, OD_READ, &this->program_id_, 4}});
  od.add_record(OD_FLASH_STATUS, {{0, OD_READ, &this->programs_, 1}, {1, OD_READ, &this->flash_status_, 4}});
}

void ProgramDownload::loop() {
  if (App.get_loop_component_start_time() - this->phase_start_ms_ < this->delay_ms_)
    return;
  if (this->phase_ == PHASE_END_WAIT) {
    ota::OTAResponseTypes ret = this->backend_end();
    if (ret != ota::OTA_RESPONSE_OK) {
      this->fail_(ret);
      return;
    }
    ESP_LOGI(TAG, "Update finished");
#ifdef USE_OTA_STATE_LISTENER
    this->notify_state_(ota::OTA_COMPLETED, 100.0f, 0);
#endif
    this->phase_ = PHASE_REBOOT_WAIT;
    this->phase_start_ms_ = App.get_loop_component_start_time();
  } else if (this->phase_ == PHASE_REBOOT_WAIT) {
    this->phase_ = PHASE_IDLE;
    this->disable_loop();
    if (this->reboot_) {
      ESP_LOGI(TAG, "Rebooting");
      App.safe_reboot();
    }
  }
}

void ProgramDownload::dump_config() {
  ESP_LOGCONFIG(TAG,
                "CANopen OTA (OD 0x%04x, 0x%04x, 0x%04x, 0x%04x):\n"
                "  Reboot: %s",
                OD_PROGRAM_DATA, OD_PROGRAM_CONTROL, OD_PROGRAM_ID, OD_FLASH_STATUS, YESNO(this->reboot_));
}

OdResult ProgramDownload::od_write(OdStream stream, const void *buf, size_t count, size_t *done) {
  if (stream.index() == OD_PROGRAM_CONTROL)
    return this->control_write_(stream, buf, count, done);
  return this->data_write_(stream, static_cast<const uint8_t *>(buf), count, done);
}

OdResult ProgramDownload::control_write_(OdStream stream, const void *buf, size_t count, size_t *done) {
  if (count != 1)
    return OdResult::OD_RESULT_TYPE_MISMATCH;
  *done = count;
  switch (*static_cast<const uint8_t *>(buf)) {
    case CONTROL_STOP:
      return this->stop_();
    case CONTROL_START:
      return this->start_();
    case CONTROL_CLEAR:
      return this->clear_();
    default:
      return OdResult::OD_RESULT_INVALID_VALUE;
  }
}

// Ends any session. The error stays readable in 0x1F57 until the next clear.
OdResult ProgramDownload::stop_() {
  if (this->finishing_())
    return OdResult::OD_RESULT_NO_DATA;
  if (this->phase_ != PHASE_IDLE) {
    ESP_LOGW(TAG, "Update stopped");
    this->backend_abort();
#ifdef USE_OTA_STATE_LISTENER
    this->notify_state_(ota::OTA_ABORT, 0.0f, 0);
#endif
  }
  this->phase_ = PHASE_IDLE;
  this->control_ = CONTROL_STOP;
  this->program_id_ = 0;
  this->flash_status_ &= ~FLASH_IN_PROGRESS;
  return OdResult::OD_RESULT_OK;
}

OdResult ProgramDownload::start_() {
  if (this->finishing_() || this->phase_ == PHASE_CLEARED || this->phase_ == PHASE_RECEIVING)
    return OdResult::OD_RESULT_NO_DATA;
  this->control_ = CONTROL_START;
  // without a new image: just back to running. With one: end() deferred, the reply to this write goes out first
  // (on some platforms the backend reboots right away).
  if (this->phase_ == PHASE_DOWNLOADED) {
    this->phase_ = PHASE_END_WAIT;
    this->phase_start_ms_ = App.get_loop_component_start_time();
    this->enable_loop();
  }
  return OdResult::OD_RESULT_OK;
}

OdResult ProgramDownload::clear_() {
  if (this->control_ != CONTROL_STOP || this->finishing_())
    return OdResult::OD_RESULT_NO_DATA;
  if (this->phase_ != PHASE_IDLE)
    this->backend_abort();
  this->phase_ = PHASE_IDLE;
  this->program_id_ = 0;
  this->flash_status_ = 0;
  this->crc_ = 0;
  this->received_ = 0;
  ota::OTAResponseTypes ret = this->backend_begin();
  if (ret != ota::OTA_RESPONSE_OK) {
    this->fail_(ret);
    return OdResult::OD_RESULT_HARDWARE;
  }
  this->phase_ = PHASE_CLEARED;
  this->control_ = CONTROL_CLEAR;
  ESP_LOGI(TAG, "Update started");
#ifdef USE_OTA_STATE_LISTENER
  this->notify_state_(ota::OTA_STARTED, 0.0f, 0);
#endif
  return OdResult::OD_RESULT_OK;
}

OdResult ProgramDownload::data_write_(OdStream stream, const uint8_t *data, size_t count, size_t *done) {
  size_t offset = stream.offset();
  bool first = this->phase_ == PHASE_CLEARED;
  // a new transfer after a part of an image (the previous one was aborted) needs stop and clear
  bool resumes = this->phase_ == PHASE_RECEIVING && offset == this->received_;
  if (!first && !resumes) {
    this->set_error_(FLASH_NOT_CLEARED, ota::OTA_RESPONSE_OK);
    return OdResult::OD_RESULT_HARDWARE;
  }
  if (first) {
    this->phase_ = PHASE_RECEIVING;
    this->flash_status_ = FLASH_IN_PROGRESS;
  }
  if (count > 0) {
#ifdef USE_OTA_MCUBOOT
    // MCUboot's IMAGE_MAGIC 0x96f3b83d, little endian
    static constexpr uint8_t MCUBOOT_MAGIC[4] = {0x3d, 0xb8, 0xf3, 0x96};
    if (this->received_ == 0 &&
        (count < sizeof(MCUBOOT_MAGIC) || std::memcmp(data, MCUBOOT_MAGIC, sizeof(MCUBOOT_MAGIC)) != 0)) {
      ESP_LOGE(TAG, "Not an MCUboot-signed image (use zephyr.signed.bin)");
      this->fail_(ota::OTA_RESPONSE_ERROR_MAGIC);
      return OdResult::OD_RESULT_HARDWARE;
    }
#endif
    ota::OTAResponseTypes ret = this->backend_write(data, count);
    if (ret != ota::OTA_RESPONSE_OK) {
      this->fail_(ret);
      return OdResult::OD_RESULT_HARDWARE;
    }
    this->crc_ = crc32_update(this->crc_, data, count);
    size_t prev = this->received_;
    this->received_ += count;
    stream.set_offset(offset + count);
    if ((prev ^ this->received_) & ~size_t(1023)) {
      ESP_LOGI(TAG, "Received %u kB", unsigned(this->received_ >> 10));
#ifdef USE_OTA_STATE_LISTENER
      // no percentage: the node doesn't know the image size
      this->notify_state_(ota::OTA_IN_PROGRESS, 0.0f, 0);
#endif
    }
  }
  *done = count;
  // a domain's size is 0 until the last chunk, then the total (CANopenNode's end of transfer indication)
  if (stream.size() == 0)
    return OdResult::OD_RESULT_PARTIAL;
  if (this->received_ == 0) {
    this->fail_(ota::OTA_RESPONSE_ERROR_UPDATE_END);
    this->set_error_(FLASH_NO_VALID_PROGRAM, ota::OTA_RESPONSE_ERROR_UPDATE_END);
    return OdResult::OD_RESULT_HARDWARE;
  }
  this->phase_ = PHASE_DOWNLOADED;
  this->control_ = CONTROL_STOP;
  this->program_id_ = this->crc_;
  this->flash_status_ = 0;
  ESP_LOGI(TAG, "Image received: %u bytes, CRC-32 0x%08" PRIx32, unsigned(this->received_), this->program_id_);
  return OdResult::OD_RESULT_OK;
}

void ProgramDownload::set_error_(uint8_t status, ota::OTAResponseTypes response) {
  this->flash_status_ = (uint32_t(status) << 1) | (uint32_t(response) << 16);
}

void ProgramDownload::fail_(ota::OTAResponseTypes error) {
  ESP_LOGE(TAG, "Update failed: %d", static_cast<int>(error));
  this->backend_abort();
  this->phase_ = PHASE_IDLE;
  this->control_ = CONTROL_STOP;
  this->program_id_ = 0;
  uint8_t status = FLASH_UNSPECIFIED;
  if (error == ota::OTA_RESPONSE_ERROR_MAGIC) {
    status = FLASH_FORMAT_UNKNOWN;
  } else if (error == ota::OTA_RESPONSE_ERROR_WRITING_FLASH) {
    status = FLASH_WRITE_ERROR;
  }
  this->set_error_(status, error);
  this->disable_loop();
#ifdef USE_OTA_STATE_LISTENER
  this->notify_state_(ota::OTA_ERROR, 0.0f, static_cast<uint8_t>(error));
#endif
}

}  // namespace esphome::canopen

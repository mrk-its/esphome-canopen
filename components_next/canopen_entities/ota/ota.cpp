#include "ota.h"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::canopen_entities {

ESPHOME_LOG_TAG(TAG, "canopen_entities.ota");

using canopen::OD_READ;
using canopen::OD_READ_WRITE;
using canopen::OD_WRITE;
using canopen::OdResult;

static constexpr uint8_t SUB_COMMAND = 1;
static constexpr uint8_t SUB_SIZE = 2;
static constexpr uint8_t SUB_MD5 = 3;
static constexpr uint8_t SUB_DATA = 4;
static constexpr uint8_t SUB_FLAGS = 5;
static constexpr uint32_t FLAG_COMPRESSION = 1;
// end() after the SDO transfer has finished, the reboot after end() (the old component's timing)
static constexpr uint32_t END_DELAY_MS = 1000;
static constexpr uint32_t REBOOT_DELAY_MS = 1000;

void CanopenOTAComponent::set_canopen(canopen::CANopenComponent *hub) {
#ifdef OTA_COMPRESSION
  this->flags_ |= FLAG_COMPRESSION;
#endif
  hub->get_od().add_record(OD_INDEX, {{0, OD_READ, &this->highest_, 1},
                                      {SUB_COMMAND, OD_WRITE, &this->command_, 4},
                                      {SUB_SIZE, OD_READ_WRITE, &this->size_, 4},
                                      {SUB_MD5, OD_WRITE, this->md5_, sizeof(this->md5_)},
                                      // a domain without storage: the handler takes it in chunks
                                      {SUB_DATA, OD_WRITE, nullptr, 0},
                                      {SUB_FLAGS, OD_READ, &this->flags_, 4}});
  hub->get_od().set_object(OD_INDEX, this);
}

void CanopenOTAComponent::setup() {
  this->backend_ = ota::make_ota_backend();
  // loop() only runs between the end of a transfer and the reboot
  this->disable_loop();
}

void CanopenOTAComponent::dump_config() {
  ESP_LOGCONFIG(TAG,
                "CANopen OTA (OD 0x%04x):\n"
                "  Compression: %s\n"
                "  Reboot: %s",
                OD_INDEX, YESNO(this->flags_ & FLAG_COMPRESSION), YESNO(this->reboot_));
}

OdResult CanopenOTAComponent::od_write(canopen::OdStream stream, const void *buf, size_t count, size_t *done) {
  if (stream.sub() != SUB_DATA)
    return stream.write_original(buf, count, done);
  size_t offset = stream.offset();
  if (offset == 0) {
    // a new image (its size from 0x3000:2)
    ota::OTAResponseTypes ret = this->begin_();
    if (ret != ota::OTA_RESPONSE_OK) {
      this->fail_(ret);
      return OdResult::OD_RESULT_HARDWARE;
    }
  } else if (this->state_ != STATE_RECEIVING) {
    return OdResult::OD_RESULT_DEVICE_STATE;
  }
  ota::OTAResponseTypes ret = this->write_(static_cast<const uint8_t *>(buf), count);
  if (ret != ota::OTA_RESPONSE_OK) {
    this->fail_(ret);
    return OdResult::OD_RESULT_HARDWARE;
  }
  size_t prev = this->received_;
  this->received_ += count;
  stream.set_offset(offset + count);
  *done = count;
  if ((prev ^ this->received_) & ~size_t(1023)) {
    // compressed: an estimate (the received bytes are compressed, size_ is not)
    float progress = std::min(this->received_ * 100.0f / this->size_, 100.0f);
    ESP_LOGI(TAG, "Received %u kB (%.0f%%)", unsigned(this->received_ >> 10), progress);
#ifdef USE_OTA_STATE_LISTENER
    this->notify_state_(ota::OTA_IN_PROGRESS, progress, 0);
#endif
  }
  // a domain's size is 0 until the last chunk, then the total (CANopenNode's end of transfer indication)
  if (stream.size() == 0)
    return OdResult::OD_RESULT_PARTIAL;
  ret = this->finish_();
  if (ret != ota::OTA_RESPONSE_OK) {
    this->fail_(ret);
    return OdResult::OD_RESULT_HARDWARE;
  }
  return OdResult::OD_RESULT_OK;
}

ota::OTAResponseTypes CanopenOTAComponent::begin_() {
  if (this->state_ == STATE_RECEIVING)
    this->backend_->abort();
  this->received_ = 0;
  if (this->size_ == 0) {
    ESP_LOGE(TAG, "Image size not set (0x3000:2)");
    return ota::OTA_RESPONSE_ERROR_UNKNOWN;
  }
  ESP_LOGI(TAG, "Starting update, %" PRIu32 " bytes", this->size_);
#ifdef USE_OTA_STATE_LISTENER
  this->notify_state_(ota::OTA_STARTED, 0.0f, 0);
#endif
#ifdef OTA_COMPRESSION
  if (this->stream_.state != nullptr)
    mz_inflateEnd(&this->stream_);
  this->stream_ = z_stream{};
  if (this->inflate_buf_ == nullptr)
    this->inflate_buf_ = std::make_unique<uint8_t[]>(INFLATE_BUF_SIZE);
  this->stream_.next_out = this->inflate_buf_.get();
  this->stream_.avail_out = INFLATE_BUF_SIZE;
  if (mz_inflateInit(&this->stream_) != MZ_OK) {
    ESP_LOGE(TAG, "Can't initialize the decompressor");
    return ota::OTA_RESPONSE_ERROR_UNKNOWN;
  }
#endif
  this->check_header_ = true;
  ota::OTAResponseTypes ret = this->backend_->begin(this->size_);
  if (ret == ota::OTA_RESPONSE_OK)
    this->state_ = STATE_RECEIVING;
  return ret;
}

#ifdef OTA_COMPRESSION
// inflates what's in stream_ into the backend; finish: until the end of the zlib stream
ota::OTAResponseTypes CanopenOTAComponent::inflate_(bool finish) {
  for (;;) {
    int status = mz_inflate(&this->stream_, MZ_SYNC_FLUSH);
    size_t n = INFLATE_BUF_SIZE - this->stream_.avail_out;
    if (n > 0 && (status == MZ_STREAM_END || this->stream_.avail_out == 0 || finish)) {
      ota::OTAResponseTypes ret = this->backend_->write(this->inflate_buf_.get(), n);
      if (ret != ota::OTA_RESPONSE_OK)
        return ret;
      this->stream_.next_out = this->inflate_buf_.get();
      this->stream_.avail_out = INFLATE_BUF_SIZE;
    }
    if (status == MZ_STREAM_END)
      return ota::OTA_RESPONSE_OK;
    if (status != MZ_OK && status != MZ_BUF_ERROR) {
      ESP_LOGE(TAG, "Decompression failed: %d", status);
      return ota::OTA_RESPONSE_ERROR_UNKNOWN;
    }
    // input consumed and the output buffer not full: wait for more (or, finishing, a truncated stream)
    if (this->stream_.avail_in == 0 && this->stream_.avail_out != 0) {
      if (finish) {
        ESP_LOGE(TAG, "Compressed image truncated");
        return ota::OTA_RESPONSE_ERROR_UNKNOWN;
      }
      return ota::OTA_RESPONSE_OK;
    }
  }
}
#endif

ota::OTAResponseTypes CanopenOTAComponent::write_(const uint8_t *data, size_t len) {
#if defined(USE_ZEPHYR) && !defined(USE_ZEPHYR_VARIANT_NATIVE_SIM)
  if (this->check_header_) {
    this->check_header_ = false;
    // MCUboot IMAGE_MAGIC 0x96f3b83d, little endian (not checked for compressed data)
    static constexpr uint8_t MCUBOOT_MAGIC[4] = {0x3d, 0xb8, 0xf3, 0x96};
    if ((this->flags_ & FLAG_COMPRESSION) == 0 &&
        (len < sizeof(MCUBOOT_MAGIC) || std::memcmp(data, MCUBOOT_MAGIC, sizeof(MCUBOOT_MAGIC)) != 0)) {
      ESP_LOGE(TAG, "Not an MCUboot-signed image (use zephyr.signed.bin)");
      return ota::OTA_RESPONSE_ERROR_MAGIC;
    }
  }
#endif
#ifdef OTA_COMPRESSION
  this->stream_.next_in = data;
  this->stream_.avail_in = len;
  return this->inflate_(false);
#else
  return this->backend_->write(const_cast<uint8_t *>(data), len);
#endif
}

ota::OTAResponseTypes CanopenOTAComponent::finish_() {
#ifdef OTA_COMPRESSION
  ota::OTAResponseTypes ret = this->inflate_(true);
  mz_inflateEnd(&this->stream_);
  if (ret != ota::OTA_RESPONSE_OK)
    return ret;
#endif
  char md5[33];
  for (size_t i = 0; i < sizeof(this->md5_); i++)
    std::snprintf(md5 + 2 * i, 3, "%02x", this->md5_[i]);
  ESP_LOGI(TAG, "Image received, MD5 %s", md5);
  this->backend_->set_update_md5(md5);
  // backend end() from loop(): on some platforms it reboots right away, before the SDO transfer has ended
  this->state_ = STATE_END_WAIT;
  this->state_start_ms_ = millis();
  this->enable_loop();
  return ota::OTA_RESPONSE_OK;
}

void CanopenOTAComponent::fail_(ota::OTAResponseTypes error) {
  ESP_LOGE(TAG, "Update failed: %d", static_cast<int>(error));
  this->backend_->abort();
#ifdef OTA_COMPRESSION
  if (this->stream_.state != nullptr)
    mz_inflateEnd(&this->stream_);
#endif
  this->state_ = STATE_IDLE;
  this->disable_loop();
#ifdef USE_OTA_STATE_LISTENER
  this->notify_state_(ota::OTA_ERROR, 0.0f, static_cast<uint8_t>(error));
#endif
}

void CanopenOTAComponent::loop() {
  switch (this->state_) {
    case STATE_END_WAIT: {
      if (millis() - this->state_start_ms_ < END_DELAY_MS)
        return;
      ota::OTAResponseTypes ret = this->backend_->end();
      if (ret != ota::OTA_RESPONSE_OK) {
        this->fail_(ret);
        return;
      }
      ESP_LOGI(TAG, "Update finished");
#ifdef USE_OTA_STATE_LISTENER
      this->notify_state_(ota::OTA_COMPLETED, 100.0f, 0);
#endif
      this->state_ = STATE_REBOOT_WAIT;
      this->state_start_ms_ = millis();
      return;
    }
    case STATE_REBOOT_WAIT:
      if (millis() - this->state_start_ms_ < REBOOT_DELAY_MS)
        return;
      this->state_ = STATE_IDLE;
      this->disable_loop();
      if (this->reboot_) {
        ESP_LOGI(TAG, "Rebooting");
        App.safe_reboot();
      }
      return;
    default:
      return;
  }
}

}  // namespace esphome::canopen_entities

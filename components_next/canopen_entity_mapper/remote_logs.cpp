#include "remote_logs.h"

#ifdef USE_CANOPEN_ENTITY_MAPPER_LOGS

#include <cinttypes>

#include "esphome/components/logger/logger.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::canopen_entity_mapper {

ESPHOME_LOG_TAG(TAG, "canopen_entity_mapper.logs");

using canopen::OD_READ;
using canopen::OD_READ_WRITE;
using canopen::OdResult;

static constexpr uint8_t SUB_DATA = 1;
static constexpr uint8_t SUB_PENDING = 2;
static constexpr uint8_t SUB_LEVEL = 3;
static constexpr uint8_t SUB_DROPPED = 4;
static constexpr uint8_t SUB_DEFAULT_LEVEL = 5;
static constexpr uint8_t SUB_TIMEOUT = 6;
static constexpr uint8_t SUB_BLOCK_SIZE = 7;

void RemoteLogs::set_canopen(canopen::CANopenComponent *hub) {
  hub->get_od().add_record(OD_INDEX, {{0, OD_READ, &this->highest_, 1},
                                      // a domain without storage: the handler reads it out of the buffer
                                      {SUB_DATA, OD_READ, nullptr, 0},
                                      {SUB_PENDING, OD_READ, &this->pending_, 4},
                                      {SUB_LEVEL, OD_READ_WRITE, &this->level_, 1},
                                      {SUB_DROPPED, OD_READ, &this->dropped_, 4},
                                      {SUB_DEFAULT_LEVEL, OD_READ, &this->default_level_, 1},
                                      {SUB_TIMEOUT, OD_READ, &this->timeout_ms_, 4},
                                      {SUB_BLOCK_SIZE, OD_READ, &this->block_size_, 1}});
  hub->get_od().set_object(OD_INDEX, this);
}

void RemoteLogs::start() {
  this->buffer_.set_timeout(this->timeout_ms_);
  this->buffer_.start(this->default_level_, millis());
  logger::global_logger->add_log_callback(
      this, [](void *self, uint8_t level, const char *tag, const char *message, size_t message_len) {
        static_cast<RemoteLogs *>(self)->buffer_.on_log(level, tag, message, message_len);
      });
}

void RemoteLogs::setup() { this->update_loop_(); }

void RemoteLogs::update_loop_() {
  if (this->buffer_.level() != 0 && this->timeout_ms_ != 0) {
    this->enable_loop();
  } else {
    this->disable_loop();
  }
}

void RemoteLogs::loop() {
  if (this->buffer_.check_timeout(millis())) {
    // logged after turning off: not buffered
    ESP_LOGI(TAG, "Remote logging off (no poll for %" PRIu32 " ms)", this->timeout_ms_);
    this->disable_loop();
  }
}

void RemoteLogs::dump_config() {
  ESP_LOGCONFIG(TAG,
                "CANopen remote logs (OD 0x%04x):\n"
                "  Buffer: %u bytes\n"
                "  Level at boot: %u (0 = off)\n"
                "  Timeout: %" PRIu32 " ms (0 = never)\n"
                "  Block size: %u",
                OD_INDEX, unsigned(this->storage_.size()), this->default_level_, this->timeout_ms_, this->block_size_);
}

OdResult RemoteLogs::od_read(canopen::OdStream stream, void *buf, size_t count, size_t *done) {
  switch (stream.sub()) {
    case SUB_DATA: {
      // the first read (at the upload's initiate) announces the snapshot; a zero-length upload is aborted by the SDO
      // server (the host reads Log Pending first)
      if (stream.offset() == 0)
        stream.set_size(this->buffer_.begin_upload());
      size_t n = this->buffer_.read(static_cast<uint8_t *>(buf), count);
      this->buffer_.keepalive(millis());
      stream.set_offset(stream.offset() + n);
      *done = n;
      return this->buffer_.upload_remaining() != 0 ? OdResult::OD_RESULT_PARTIAL : OdResult::OD_RESULT_OK;
    }
    case SUB_PENDING:
      this->pending_ = this->buffer_.pending();
      this->buffer_.keepalive(millis());
      break;
    case SUB_LEVEL:
      this->level_ = this->buffer_.level();
      break;
    case SUB_DROPPED:
      this->dropped_ = this->buffer_.dropped();
      break;
    default:
      break;
  }
  return stream.read_original(buf, count, done);
}

OdResult RemoteLogs::od_write(canopen::OdStream stream, const void *buf, size_t count, size_t *done) {
  if (stream.sub() != SUB_LEVEL)
    return stream.write_original(buf, count, done);
  if (count != 1)
    return count > 1 ? OdResult::OD_RESULT_DATA_LONG : OdResult::OD_RESULT_DATA_SHORT;
  // the subscription and its keepalive
  if (!this->buffer_.set_level(*static_cast<const uint8_t *>(buf), millis()))
    return OdResult::OD_RESULT_VALUE_HIGH;
  this->level_ = this->buffer_.level();
  *done = 1;
  this->update_loop_();
  return OdResult::OD_RESULT_OK;
}

}  // namespace esphome::canopen_entity_mapper

#endif  // USE_CANOPEN_ENTITY_MAPPER_LOGS

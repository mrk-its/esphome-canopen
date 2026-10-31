#include "log_buffer.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/log.h"
#include "esphome/core/progmem.h"

namespace esphome::canopen_entity_mapper {

static constexpr char DROP_MARKER_PREFIX[] = "[W][canopen.logs]: ";
static constexpr char DROP_MARKER_SUFFIX[] = " messages dropped";
static constexpr size_t DROP_MARKER_MAX = sizeof(DROP_MARKER_PREFIX) + sizeof(DROP_MARKER_SUFFIX) + 10;

void LogBuffer::clear() {
  this->tail_ = 0;
  this->pending_ = 0;
  this->upload_remaining_ = 0;
  this->unreported_drops_ = 0;
}

bool LogBuffer::set_level(uint8_t level, uint32_t now_ms) {
  if (level > ESPHOME_LOG_LEVEL_VERY_VERBOSE)
    return false;
  if (this->level_ == 0 && level != 0)
    this->clear();
  this->level_ = level;
  this->last_poll_ms_ = now_ms;
  return true;
}

bool LogBuffer::check_timeout(uint32_t now_ms) {
  if (this->level_ == 0 || this->timeout_ms_ == 0 || now_ms - this->last_poll_ms_ <= this->timeout_ms_)
    return false;
  this->level_ = 0;
  this->clear();
  return true;
}

void LogBuffer::put_(const char *data, size_t len) {
  uint32_t head = (this->tail_ + this->pending_) % this->size_;
  size_t first = std::min<size_t>(this->size_ - head, len);
  std::memcpy(this->buf_ + head, data, first);
  std::memcpy(this->buf_, data + first, len - first);
  this->pending_ += len;
}

// "<prefix><n><suffix>\n" into out (DROP_MARKER_MAX bytes), returns its length
static size_t format_drop_marker(char *out, uint32_t n) {
  size_t pos = sizeof(DROP_MARKER_PREFIX) - 1;
  std::memcpy(out, DROP_MARKER_PREFIX, pos);
  char digits[10];
  size_t num = 0;
  do {
    digits[num++] = char('0' + n % 10);
    n /= 10;
  } while (n != 0);
  while (num != 0)
    out[pos++] = digits[--num];
  std::memcpy(out + pos, DROP_MARKER_SUFFIX, sizeof(DROP_MARKER_SUFFIX) - 1);
  pos += sizeof(DROP_MARKER_SUFFIX) - 1;
  out[pos++] = '\n';
  return pos;
}

void LogBuffer::on_log(uint8_t msg_level, const char *tag, const char *message, size_t len) {
  if (this->level_ == 0 || msg_level > this->level_)
    return;
  // canbus logs every frame at DEBUG, and the CANopen stack's own debug logs would feed the log uploads; the tag may
  // be in flash (ESP8266)
  if (msg_level > ESPHOME_LOG_LEVEL_INFO &&
      (ESPHOME_strcmp_P("canbus", tag) == 0 || ESPHOME_strcmp_P("canopen", tag) == 0))
    return;
  while (len != 0 && (message[len - 1] == '\n' || message[len - 1] == '\0'))
    len--;
  len = std::min<size_t>(len, this->size_ - 1);

  // the newest whole line is dropped when it doesn't fit, the gap is marked in the stream once there is room again
  char marker[DROP_MARKER_MAX];
  size_t marker_len = this->unreported_drops_ != 0 ? format_drop_marker(marker, this->unreported_drops_) : 0;
  if (marker_len + len + 1 > this->size_ - this->pending_) {
    this->dropped_++;
    this->unreported_drops_++;
    return;
  }
  if (marker_len != 0) {
    this->put_(marker, marker_len);
    this->unreported_drops_ = 0;
  }
  this->put_(message, len);
  this->put_("\n", 1);
}

size_t LogBuffer::read(uint8_t *out, size_t count) {
  size_t n = std::min<size_t>(count, this->upload_remaining_);
  size_t first = std::min<size_t>(this->size_ - this->tail_, n);
  std::memcpy(out, this->buf_ + this->tail_, first);
  std::memcpy(out + first, this->buf_, n - first);
  this->tail_ = (this->tail_ + n) % this->size_;
  this->pending_ -= n;
  this->upload_remaining_ -= n;
  return n;
}

}  // namespace esphome::canopen_entity_mapper

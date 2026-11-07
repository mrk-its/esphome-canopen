#include "log_buffer.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/log.h"
#include "esphome/core/progmem.h"

namespace esphome::canopen_entities {

static constexpr char DROP_MARKER_PREFIX[] = "[W][canopen.logs]: ";
static constexpr char DROP_MARKER_SUFFIX[] = " messages dropped";
static_assert(sizeof(DROP_MARKER_PREFIX) + sizeof(DROP_MARKER_SUFFIX) + 10 <= 64, "MARKER_MAX");

// "<prefix><n><suffix>\n" into out (MARKER_MAX bytes), returns its length
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

bool LogBuffer::set_level(uint8_t level, uint32_t now_ms) {
  if (level > ESPHOME_LOG_LEVEL_VERY_VERBOSE)
    return false;
  this->level_ = level != 0 ? level : this->default_level_;
  this->last_poll_ms_ = now_ms;
  return true;
}

bool LogBuffer::check_timeout(uint32_t now_ms) {
  if (this->timeout_ms_ == 0 || now_ms - this->last_poll_ms_ <= this->timeout_ms_)
    return false;
  if (this->upload_in_progress())
    this->cancel_upload_();
  if (this->level_ == this->default_level_)
    return false;
  this->level_ = this->default_level_;
  return true;
}

void LogBuffer::cancel_upload_() {
  // unreported_drops_ are lines lost behind the snapshot during this upload: the next upload reports them at its
  // start, so the unread rest of the snapshot (older) goes too
  if (this->unreported_drops_ != 0 && this->upload_buf_remaining_ != 0)
    this->evict_(0, this->upload_buf_remaining_);
  if (this->marker_pos_ < this->marker_len_)
    this->unreported_drops_ += this->marker_drops_;  // reported again by the next upload
  this->marker_len_ = 0;
  this->marker_pos_ = 0;
  this->marker_drops_ = 0;
  this->upload_buf_remaining_ = 0;
}

uint32_t LogBuffer::begin_upload() {
  this->cancel_upload_();  // the previous one, if the host gave it up
  if (this->unreported_drops_ != 0) {
    this->marker_len_ = format_drop_marker(this->marker_, this->unreported_drops_);
    this->marker_drops_ = this->unreported_drops_;
    this->unreported_drops_ = 0;
  }
  this->upload_buf_remaining_ = this->pending_;
  return this->upload_remaining();
}

uint32_t LogBuffer::pending() const {
  uint32_t drops = this->unreported_drops_ + (this->marker_pos_ < this->marker_len_ ? this->marker_drops_ : 0);
  if (drops == 0)
    return this->pending_;
  char marker[MARKER_MAX];
  return this->pending_ + format_drop_marker(marker, drops);
}

void LogBuffer::put_(const char *data, size_t len) {
  uint32_t head = (this->tail_ + this->pending_) % this->size_;
  size_t first = std::min<size_t>(this->size_ - head, len);
  std::memcpy(this->buf_ + head, data, first);
  std::memcpy(this->buf_, data + first, len - first);
  this->pending_ += len;
}

void LogBuffer::evict_(uint32_t from, uint32_t min_bytes) {
  // every buffered line ends with '\n', `from` is a line start (0, or the end of an upload's snapshot)
  uint32_t end = from;
  uint32_t lines = 0;
  while (end - from < min_bytes && end < this->pending_) {
    while (end < this->pending_ && this->at_(end) != '\n')
      end++;
    end++;
    lines++;
  }
  uint32_t n = std::min(end, this->pending_) - from;
  if (from == 0) {
    this->tail_ = (this->tail_ + n) % this->size_;
  } else {
    // behind an upload's snapshot: move the newer lines down over the evicted ones
    for (uint32_t i = from; i + n < this->pending_; i++)
      this->at_(i) = this->at_(i + n);
  }
  this->pending_ -= n;
  this->dropped_ += lines;
  this->unreported_drops_ += lines;
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

  uint32_t need = len + 1;
  uint32_t keep = this->upload_buf_remaining_;  // an upload's snapshot is never evicted
  if (need > this->size_ - keep) {
    // doesn't fit even without the lines behind the snapshot (a long line during an upload of a full buffer): the
    // new line is dropped, and the lines behind the snapshot (older) too, so the gap is at the start of the next
    // upload, where it is reported
    if (this->pending_ > keep)
      this->evict_(keep, this->pending_ - keep);
    this->dropped_++;
    this->unreported_drops_++;
    return;
  }
  uint32_t free = this->size_ - this->pending_;
  if (need > free)
    this->evict_(keep, need - free);  // the oldest lines (behind the snapshot during an upload)
  this->put_(message, len);
  this->put_("\n", 1);
}

size_t LogBuffer::read(uint8_t *out, size_t count) {
  size_t n = std::min<size_t>(count, this->marker_len_ - this->marker_pos_);
  std::memcpy(out, this->marker_ + this->marker_pos_, n);
  this->marker_pos_ += n;
  size_t m = std::min<size_t>(count - n, this->upload_buf_remaining_);
  size_t first = std::min<size_t>(this->size_ - this->tail_, m);
  std::memcpy(out + n, this->buf_ + this->tail_, first);
  std::memcpy(out + n + first, this->buf_, m - first);
  this->tail_ = (this->tail_ + m) % this->size_;
  this->pending_ -= m;
  this->upload_buf_remaining_ -= m;
  return n + m;
}

}  // namespace esphome::canopen_entities

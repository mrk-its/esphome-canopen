#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::canopen_entity_mapper {

// Remote logs (OD 0x3100, OBJECT_DICTIONARY.md) without the OD glue: a ring buffer of log lines, the level the host
// subscribed with and its keepalive. The host consumes the lines with uploads of Log Data (begin_upload(), read()).
// Everything runs on the main loop (log callbacks and SDO processing both do), so there is no locking. Nothing here
// may log: a log call would feed back into on_log() in the middle of a read.
class LogBuffer {
 public:
  // `buf` / `size`: the storage, owned by the caller
  LogBuffer(uint8_t *buf, uint32_t size) : buf_(buf), size_(size) {}

  // the level at boot (0 = off), starts the keepalive
  void start(uint8_t level, uint32_t now_ms) {
    this->level_ = level;
    this->last_poll_ms_ = now_ms;
  }
  void set_timeout(uint32_t timeout_ms) { this->timeout_ms_ = timeout_ms; }  // 0 = never

  // a logger callback: one formatted line, without or with a trailing '\n'
  void on_log(uint8_t msg_level, const char *tag, const char *message, size_t len);

  uint8_t level() const { return this->level_; }
  // a host write of Log Level (keepalive): 0 = off, 1..7 = ESPHOME_LOG_LEVEL_ERROR..VERY_VERBOSE. 0 -> >0 clears the
  // buffer, rewriting a level keeps it. False if the level is invalid (nothing changes).
  bool set_level(uint8_t level, uint32_t now_ms);
  void keepalive(uint32_t now_ms) { this->last_poll_ms_ = now_ms; }
  // turns logging off (and clears the buffer) when no host kept it alive for the timeout; true if it did
  bool check_timeout(uint32_t now_ms);
  uint32_t timeout_ms() const { return this->timeout_ms_; }

  // An upload of Log Data: begin_upload() takes a snapshot of the buffered bytes (the size the upload announces),
  // read() copies and consumes at most what is left of it (the SDO server may ask for more), so lines logged during
  // the transfer stay for the next one.
  uint32_t begin_upload() {
    this->upload_remaining_ = this->pending_;
    return this->upload_remaining_;
  }
  size_t read(uint8_t *out, size_t count);
  uint32_t upload_remaining() const { return this->upload_remaining_; }

  uint32_t pending() const { return this->pending_; }  // buffered bytes
  uint32_t dropped() const { return this->dropped_; }  // lines dropped because the buffer was full, since boot
  uint32_t unreported_drops() const { return this->unreported_drops_; }
  void clear();

 protected:
  void put_(const char *data, size_t len);

  uint8_t *buf_;
  uint32_t size_;
  uint32_t tail_{0};     // read position
  uint32_t pending_{0};  // head = (tail_ + pending_) % size_
  uint32_t upload_remaining_{0};
  uint32_t dropped_{0};
  uint32_t unreported_drops_{0};  // drops not yet marked in the stream
  uint32_t timeout_ms_{0};
  uint32_t last_poll_ms_{0};
  uint8_t level_{0};
};

}  // namespace esphome::canopen_entity_mapper

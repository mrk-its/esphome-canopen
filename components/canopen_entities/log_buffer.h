#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::canopen_entities {

// Remote logs (OD 0x3100, OBJECT_DICTIONARY.md) without the OD glue: a ring buffer of log lines, the recording level
// and the host's keepalive. The host consumes the lines with uploads of Log Data (begin_upload(), read()).
//
// A flight recorder: the node records at its default level (`remote_logs: level:`) from boot, whether a host reads
// or not. When the buffer is full the oldest lines are evicted, so a host subscribing later gets the most recent
// lines, preceded by `[W][canopen.logs]: N messages dropped`. A host may change the level while it keeps polling;
// without a poll for the timeout, or when it writes 0, the level returns to the default.
//
// Everything runs on the main loop (log callbacks and SDO processing both do), so there is no locking. Nothing here
// may log: a log call would feed back into on_log() in the middle of a read.
class LogBuffer {
 public:
  // `buf` / `size`: the storage, owned by the caller
  LogBuffer(uint8_t *buf, uint32_t size) : buf_(buf), size_(size) {}

  // the default level (0 = nothing recorded until a host writes a level), starts the keepalive
  void start(uint8_t default_level, uint32_t now_ms) {
    this->default_level_ = default_level;
    this->level_ = default_level;
    this->last_poll_ms_ = now_ms;
  }
  void set_timeout(uint32_t timeout_ms) { this->timeout_ms_ = timeout_ms; }  // 0 = never

  // a logger callback: one formatted line, without or with a trailing '\n'
  void on_log(uint8_t msg_level, const char *tag, const char *message, size_t len);

  uint8_t level() const { return this->level_; }
  uint8_t default_level() const { return this->default_level_; }
  // a host write of Log Level (keepalive): 1..7 = ESPHOME_LOG_LEVEL_ERROR..VERY_VERBOSE, 0 = back to the default
  // level. The buffer is kept. False if the level is invalid (nothing changes).
  bool set_level(uint8_t level, uint32_t now_ms);
  void keepalive(uint32_t now_ms) { this->last_poll_ms_ = now_ms; }
  // no host kept it alive for the timeout: the level returns to the default, and an upload the host abandoned is
  // given up (its snapshot becomes evictable again, the unread part stays for the next upload). True if the level
  // changed.
  bool check_timeout(uint32_t now_ms);
  uint32_t timeout_ms() const { return this->timeout_ms_; }
  // check_timeout() has something to do: a level set by a host, or an upload in progress
  bool needs_timeout_check() const {
    return this->timeout_ms_ != 0 && (this->level_ != this->default_level_ || this->upload_in_progress());
  }

  // An upload of Log Data: begin_upload() takes a snapshot of the buffered bytes, plus the drop marker if lines were
  // evicted (the size the upload announces); read() copies and consumes at most what is left of it (the SDO server
  // may ask for more), so lines logged during the transfer stay for the next one. The snapshot is never evicted.
  uint32_t begin_upload();
  size_t read(uint8_t *out, size_t count);
  uint32_t upload_remaining() const { return (this->marker_len_ - this->marker_pos_) + this->upload_buf_remaining_; }
  bool upload_in_progress() const { return this->upload_remaining() != 0; }

  // what the next upload would announce: the buffered bytes plus the drop marker
  uint32_t pending() const;
  uint32_t dropped() const { return this->dropped_; }  // lines evicted or dropped since boot
  uint32_t unreported_drops() const { return this->unreported_drops_; }

 protected:
  void put_(const char *data, size_t len);
  uint8_t &at_(uint32_t offset) { return this->buf_[(this->tail_ + offset) % this->size_]; }
  // removes whole lines from buffer offset `from` on, at least `min_bytes` of them
  void evict_(uint32_t from, uint32_t min_bytes);
  // gives up the current upload; drops a marker not sent completely back into unreported_drops_
  void cancel_upload_();

  static constexpr size_t MARKER_MAX = 64;

  uint8_t *buf_;
  uint32_t size_;
  uint32_t tail_{0};                  // read position
  uint32_t pending_{0};               // buffered bytes, head = (tail_ + pending_) % size_
  uint32_t upload_buf_remaining_{0};  // snapshot bytes of the current upload not read yet, from tail_ on
  char marker_[MARKER_MAX]{};         // the drop marker of the current upload, sent before its snapshot
  uint32_t marker_len_{0};
  uint32_t marker_pos_{0};
  uint32_t marker_drops_{0};  // the count in marker_
  uint32_t dropped_{0};
  uint32_t unreported_drops_{0};  // evicted / dropped lines not in a marker yet
  uint32_t timeout_ms_{0};
  uint32_t last_poll_ms_{0};
  uint8_t level_{0};
  uint8_t default_level_{0};
};

}  // namespace esphome::canopen_entities

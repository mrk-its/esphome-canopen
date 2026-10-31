// Host test of components/canopen_entities/log_buffer.cpp (remote logs, OD 0x3100), built and run by
// test/test_logs.py. Uploads are read the way the mapper's od_read() serves them to CANopenNode's SDO server:
// begin_upload() at the first read, then read() for the free buffer space, which may be more than is left.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "esphome/core/log.h"
#include "log_buffer.h"

using esphome::canopen_entities::LogBuffer;

#define CHECK(cond) \
  do { \
    if (!(cond)) { \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      exit(1); \
    } \
  } while (0)

static constexpr uint32_t SIZE = 256;
static uint8_t storage[SIZE];
static LogBuffer logs(storage, SIZE);
static uint32_t now_ms = 0;
static bool log_during_upload = false;

// like a logger callback: the logger passes the formatted line without '\n'
static void log(uint8_t level, const char *tag, const std::string &msg) {
  logs.on_log(level, tag, msg.c_str(), msg.size());
}

// one upload of Log Data in reads of `chunk` bytes, as many as the SDO server asks for
static std::string upload(uint32_t chunk) {
  uint32_t size = logs.begin_upload();
  std::string out;
  while (out.size() < size) {
    uint8_t b[900];  // the SDO server buffer with block transfer
    CHECK(chunk <= sizeof(b));
    size_t n = logs.read(b, chunk);
    CHECK(n == std::min<size_t>(chunk, size - out.size()));  // never more than announced
    out.append(reinterpret_cast<char *>(b), n);
    if (log_during_upload)
      log(ESPHOME_LOG_LEVEL_DEBUG, "x", "MID");
  }
  CHECK(logs.upload_remaining() == 0);
  return out;
}

static void drain() {
  while (logs.pending() != 0)
    upload(21);
}

static void test_filters() {
  log(ESPHOME_LOG_LEVEL_VERBOSE, "x", "verbose");   // above the level
  log(ESPHOME_LOG_LEVEL_DEBUG, "canbus", "frame");  // canbus below INFO
  log(ESPHOME_LOG_LEVEL_DEBUG, "canopen", "sdo");   // canopen below INFO
  log(ESPHOME_LOG_LEVEL_INFO, "canbus", "canbus info");
  log(ESPHOME_LOG_LEVEL_DEBUG, "x", "dbg");
  CHECK(upload(21) == "canbus info\ndbg\n");
}

static void test_lines_logged_during_upload_are_kept() {
  log(ESPHOME_LOG_LEVEL_DEBUG, "x", std::string(50, 'a'));
  log_during_upload = true;
  CHECK(upload(21) == std::string(50, 'a') + "\n");  // 3 reads of 21: the last one must not eat MID lines
  log_during_upload = false;
  CHECK(logs.pending() == 3 * 4);
  CHECK(upload(7) == "MID\nMID\nMID\n");
}

static void test_read_asks_for_more_than_left() {
  log(ESPHOME_LOG_LEVEL_INFO, "t", "short");
  CHECK(logs.begin_upload() == 6);
  log(ESPHOME_LOG_LEVEL_INFO, "t", "later");
  uint8_t b[100];
  CHECK(logs.read(b, sizeof(b)) == 6);  // the SDO server's free buffer space
  CHECK(logs.read(b, sizeof(b)) == 0);
  CHECK(std::string(reinterpret_cast<char *>(b), 5) == "short");
  CHECK(upload(SIZE) == "later\n");
}

static void test_wrap_around() {
  for (int i = 0; i < 200; i++) {
    std::string line = "line " + std::to_string(i) + std::string(i % 37, 'y');
    log(ESPHOME_LOG_LEVEL_INFO, "t", line);
    log(ESPHOME_LOG_LEVEL_INFO, "t", line + "\n");  // a trailing '\n' from the logger is not doubled
    CHECK(upload(i % 2 ? 7 : 889) == line + "\n" + line + "\n");
  }
}

static void test_overflow_marker() {
  uint32_t dropped = logs.dropped();
  for (int i = 0; i < 20; i++)
    log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(30, 'a' + i));  // 31 bytes each, 8 fit in 256
  CHECK(logs.pending() == 8 * 31);
  CHECK(logs.dropped() - dropped == 12);
  std::string first = upload(SIZE);
  CHECK(first.find(std::string(30, 'a' + 7)) != std::string::npos);  // oldest lines kept
  log(ESPHOME_LOG_LEVEL_INFO, "t", "after");
  CHECK(upload(SIZE) == "[W][canopen.logs]: 12 messages dropped\nafter\n");
}

static void test_marker_needs_room_for_the_line_too() {
  for (uint32_t i = 0; i < SIZE / 32; i++)
    log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(31, 'b'));  // fills the buffer exactly
  log(ESPHOME_LOG_LEVEL_INFO, "t", "dropped 1");
  CHECK(logs.unreported_drops() == 1);
  // free 40 bytes: the marker (38) fits, marker + line doesn't, both are skipped
  uint8_t b[40];
  logs.begin_upload();
  CHECK(logs.read(b, sizeof(b)) == sizeof(b));
  log(ESPHOME_LOG_LEVEL_INFO, "t", "dropped 2");
  CHECK(logs.unreported_drops() == 2);
  drain();
  log(ESPHOME_LOG_LEVEL_INFO, "t", "x");
  CHECK(upload(SIZE) == "[W][canopen.logs]: 2 messages dropped\nx\n");
}

static void test_long_line_truncated() {
  log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(1000, 'z'));
  CHECK(logs.pending() == SIZE);
  CHECK(upload(SIZE) == std::string(SIZE - 1, 'z') + "\n");
}

static void test_level_writes() {
  log(ESPHOME_LOG_LEVEL_INFO, "t", "keep");
  CHECK(logs.set_level(ESPHOME_LOG_LEVEL_DEBUG, now_ms));  // same or other level > 0: keepalive, keeps the buffer
  CHECK(logs.pending() == 5);
  CHECK(logs.set_level(0, now_ms));
  log(ESPHOME_LOG_LEVEL_ERROR, "t", "ignored while off");
  CHECK(logs.pending() == 5);
  CHECK(logs.set_level(ESPHOME_LOG_LEVEL_INFO, now_ms));  // 0 -> >0 clears
  CHECK(logs.pending() == 0);
  CHECK(logs.level() == ESPHOME_LOG_LEVEL_INFO);
  CHECK(!logs.set_level(ESPHOME_LOG_LEVEL_VERY_VERBOSE + 1, now_ms));
  CHECK(logs.level() == ESPHOME_LOG_LEVEL_INFO);
}

static void test_upload_of_empty_buffer_has_size_0() {
  drain();
  CHECK(logs.begin_upload() == 0);  // the SDO server aborts a zero-length upload
}

static void test_timeout() {
  logs.set_timeout(1000);
  CHECK(logs.set_level(ESPHOME_LOG_LEVEL_INFO, now_ms));  // keepalive
  uint32_t t0 = now_ms;
  CHECK(!logs.check_timeout(t0 + 900));
  logs.keepalive(t0 + 900);  // a Log Pending read or a Log Data upload
  CHECK(!logs.check_timeout(t0 + 1800));
  log(ESPHOME_LOG_LEVEL_INFO, "t", "lost on auto-off");
  CHECK(logs.check_timeout(t0 + 1901));
  CHECK(logs.level() == 0 && logs.pending() == 0);
  CHECK(!logs.check_timeout(t0 + 5000));  // already off
  log(ESPHOME_LOG_LEVEL_ERROR, "t", "ignored");
  CHECK(logs.pending() == 0);

  // millis() wrap
  now_ms = 0xFFFFFF00u;
  CHECK(logs.set_level(ESPHOME_LOG_LEVEL_INFO, now_ms));
  CHECK(!logs.check_timeout(0x100));  // 512 ms later
  CHECK(logs.check_timeout(0x400));

  logs.set_timeout(0);  // never
  CHECK(logs.set_level(ESPHOME_LOG_LEVEL_INFO, now_ms));
  CHECK(!logs.check_timeout(now_ms + 0x7FFFFFFF));
}

int main() {
  logs.set_timeout(30000);
  logs.start(ESPHOME_LOG_LEVEL_DEBUG, now_ms);
  CHECK(logs.level() == ESPHOME_LOG_LEVEL_DEBUG);  // buffers from boot with a default level

  test_filters();
  test_lines_logged_during_upload_are_kept();
  test_read_asks_for_more_than_left();
  test_wrap_around();
  test_overflow_marker();
  test_marker_needs_room_for_the_line_too();
  test_long_line_truncated();
  test_level_writes();
  test_upload_of_empty_buffer_has_size_0();
  test_timeout();
  printf("OK\n");
  return 0;
}

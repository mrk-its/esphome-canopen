// Host test of components/canopen/logs.cpp (remote logs, OD 0x3100), built and run by test/test_logs.py.
// Emulates how canopen-stack calls the object types (co_ssdo.c): Size + Reset at the start of an upload, then
// Read in chunks; block uploads pass SegNum * 7 on the last block too, not the remaining size.
#include <cstdio>
#include <cstdlib>
#include <string>

#include "esphome/core/log.h"
#include "logs.h"

using namespace esphome::canopen;

namespace esphome {
uint32_t fake_millis = 0;
}

#define CHECK(cond) \
  do { \
    if (!(cond)) { \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      exit(1); \
    } \
  } while (0)

static const uint32_t SIZE = CANOPEN_LOG_BUFFER_SIZE;
static CO_OBJ obj{0, nullptr, (CO_DATA) &canopen_logs};
static CO_NODE node;
static bool log_during_upload = false;

// like a logger callback: the logger passes the formatted line without '\n'
static void log(uint8_t level, const char *tag, const std::string &msg) {
  canopen_logs.on_log(level, tag, msg.c_str(), msg.size());
}

static void set_level(uint8_t level) { CHECK(LogLevelWrite(&obj, &node, &level, 1) == CO_ERR_NONE); }

// one SDO upload of Log Data, Read in chunks of `chunk` bytes
static std::string upload(uint32_t chunk) {
  uint32_t size = LogDataSize(&obj, &node, 0);
  LogDataReset(&obj, &node, 0);
  std::string out;
  uint32_t left = size;
  while (left) {
    char b[SIZE + 8];
    CHECK(chunk <= sizeof(b));
    CHECK(LogDataRead(&obj, &node, b, chunk) == CO_ERR_NONE);
    uint32_t n = left < chunk ? left : chunk;
    for (uint32_t i = n; i < chunk; i++)
      CHECK(b[i] == 0);  // zero-filled past the announced size
    out.append(b, n);
    left -= n;
    if (log_during_upload)
      log(ESPHOME_LOG_LEVEL_DEBUG, "x", "MID");
  }
  return out;
}

static void drain() {
  while (canopen_logs.pending)
    upload(21);
}

static void test_filters() {
  log(ESPHOME_LOG_LEVEL_VERBOSE, "x", "verbose");      // above the level
  log(ESPHOME_LOG_LEVEL_DEBUG, "canbus", "frame");     // canbus below INFO
  log(ESPHOME_LOG_LEVEL_DEBUG, "canopen", "sdo");      // canopen below INFO
  log(ESPHOME_LOG_LEVEL_INFO, "canbus", "canbus info");
  log(ESPHOME_LOG_LEVEL_DEBUG, "x", "dbg");
  CHECK(upload(21) == "canbus info\ndbg\n");
}

static void test_lines_logged_during_upload_are_kept() {
  log(ESPHOME_LOG_LEVEL_DEBUG, "x", std::string(50, 'a'));
  log_during_upload = true;
  CHECK(upload(21) == std::string(50, 'a') + "\n");  // 3 reads of 21: the last one must not eat MID lines
  log_during_upload = false;
  CHECK(canopen_logs.pending == 3 * 4);
  CHECK(upload(7) == "MID\nMID\nMID\n");
}

static void test_wrap_around() {
  for (int i = 0; i < 200; i++) {
    std::string line = "line " + std::to_string(i) + std::string(i % 37, 'y');
    log(ESPHOME_LOG_LEVEL_INFO, "t", line);
    log(ESPHOME_LOG_LEVEL_INFO, "t", line + "\n");  // a trailing '\n' from the logger is not doubled
    CHECK(upload(i % 2 ? 7 : 21) == line + "\n" + line + "\n");
  }
}

static void test_overflow_marker() {
  uint32_t dropped = canopen_logs.dropped;
  for (int i = 0; i < 20; i++)
    log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(30, 'a' + i));  // 31 bytes each, 8 fit in 256
  CHECK(canopen_logs.pending == 8 * 31);
  CHECK(canopen_logs.dropped - dropped == 12);
  std::string first = upload(SIZE);
  CHECK(first.find(std::string(30, 'a' + 7)) != std::string::npos);  // oldest lines kept
  log(ESPHOME_LOG_LEVEL_INFO, "t", "after");
  CHECK(upload(SIZE) == "[W][canopen.logs]: 12 messages dropped\nafter\n");
}

static void test_marker_needs_room_for_the_line_too() {
  for (uint32_t i = 0; i < SIZE / 32; i++)
    log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(31, 'b'));  // fills the buffer exactly
  log(ESPHOME_LOG_LEVEL_INFO, "t", "dropped 1");
  CHECK(canopen_logs.unreported_drops == 1);
  // free 40 bytes: the marker (38) fits, marker + line doesn't, both are skipped
  char b[40];
  LogDataSize(&obj, &node, 0);
  canopen_logs.xfer_remaining = sizeof(b);
  LogDataRead(&obj, &node, b, sizeof(b));
  log(ESPHOME_LOG_LEVEL_INFO, "t", "dropped 2");
  CHECK(canopen_logs.unreported_drops == 2);
  drain();
  log(ESPHOME_LOG_LEVEL_INFO, "t", "x");
  CHECK(upload(SIZE) == "[W][canopen.logs]: 2 messages dropped\nx\n");
}

static void test_long_line_truncated() {
  log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(1000, 'z'));
  CHECK(canopen_logs.pending == SIZE);
  CHECK(upload(SIZE) == std::string(SIZE - 1, 'z') + "\n");
}

static void test_level_writes() {
  log(ESPHOME_LOG_LEVEL_INFO, "t", "keep");
  set_level(ESPHOME_LOG_LEVEL_DEBUG);  // same or other level > 0: keepalive, keeps the buffer
  CHECK(canopen_logs.pending == 5);
  set_level(0);
  log(ESPHOME_LOG_LEVEL_ERROR, "t", "ignored while off");
  CHECK(canopen_logs.pending == 5);
  set_level(ESPHOME_LOG_LEVEL_INFO);  // 0 -> >0 clears
  CHECK(canopen_logs.pending == 0);
  uint8_t level;
  CHECK(LogLevelRead(&obj, &node, &level, 1) == CO_ERR_NONE && level == ESPHOME_LOG_LEVEL_INFO);
  level = ESPHOME_LOG_LEVEL_VERY_VERBOSE + 1;
  CHECK(LogLevelWrite(&obj, &node, &level, 1) != CO_ERR_NONE);
}

static void test_upload_of_empty_buffer_has_size_0() {
  drain();
  CHECK(LogDataSize(&obj, &node, 0) == 0);  // canopen-stack aborts the upload
}

static void test_timeout() {
  canopen_logs.timeout_ms = 1000;
  set_level(ESPHOME_LOG_LEVEL_INFO);  // keepalive at fake_millis
  uint32_t t0 = esphome::fake_millis;
  CHECK(!canopen_logs.check_timeout(t0 + 900));
  esphome::fake_millis = t0 + 900;
  uint32_t pending;
  CHECK(LogPendingRead(&obj, &node, &pending, 4) == CO_ERR_NONE);  // keepalive
  CHECK(!canopen_logs.check_timeout(t0 + 1800));
  log(ESPHOME_LOG_LEVEL_INFO, "t", "lost on auto-off");
  CHECK(canopen_logs.check_timeout(t0 + 1901));
  CHECK(canopen_logs.level == 0 && canopen_logs.pending == 0);
  CHECK(!canopen_logs.check_timeout(t0 + 5000));  // already off
  log(ESPHOME_LOG_LEVEL_ERROR, "t", "ignored");
  CHECK(canopen_logs.pending == 0);

  // millis() wrap
  esphome::fake_millis = 0xFFFFFF00u;
  set_level(ESPHOME_LOG_LEVEL_INFO);
  CHECK(!canopen_logs.check_timeout(0x100));  // 512 ms later
  CHECK(canopen_logs.check_timeout(0x400));

  canopen_logs.timeout_ms = 0;  // never
  set_level(ESPHOME_LOG_LEVEL_INFO);
  CHECK(!canopen_logs.check_timeout(esphome::fake_millis + 0x7FFFFFFF));
}

int main() {
  canopen_logs.default_level = ESPHOME_LOG_LEVEL_DEBUG;
  canopen_logs.timeout_ms = 30000;
  canopen_logs.setup();
  CHECK(canopen_logs.level == ESPHOME_LOG_LEVEL_DEBUG);  // buffers from boot with a default level

  test_filters();
  test_lines_logged_during_upload_are_kept();
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

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

static std::string marker(uint32_t n) { return "[W][canopen.logs]: " + std::to_string(n) + " messages dropped\n"; }

static void test_overflow_evicts_oldest() {
  drain();
  uint32_t dropped = logs.dropped();
  for (int i = 0; i < 20; i++)
    log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(30, 'a' + i));  // 31 bytes each, 8 fit in 256
  CHECK(logs.dropped() - dropped == 12);
  CHECK(logs.unreported_drops() == 12);
  CHECK(logs.pending() == marker(12).size() + 8 * 31);  // Log Pending announces the marker too
  std::string expected = marker(12);
  for (int i = 12; i < 20; i++)
    expected += std::string(30, 'a' + i) + "\n";  // the newest 8 lines
  CHECK(upload(13) == expected);
  log(ESPHOME_LOG_LEVEL_INFO, "t", "after");
  CHECK(upload(SIZE) == "after\n");  // reported once
}

// lines logged during an upload of a full buffer evict the oldest lines behind its snapshot, never the snapshot
static void upload_snapshot_is_not_evicted(bool long_line) {
  drain();
  for (int i = 0; i < 8; i++)
    log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(30, 'a' + i));  // 248 bytes
  std::string snapshot;
  uint8_t b[SIZE];
  CHECK(logs.begin_upload() == 248);
  size_t n = logs.read(b, 40);
  snapshot.append(reinterpret_cast<char *>(b), n);
  // 208 snapshot bytes left: 48 free for lines behind it
  log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(9, 'X'));   // 10
  log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(9, 'Y'));   // 10
  log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(29, 'Z'));  // 30: 28 free, evicts X
  CHECK(logs.unreported_drops() == 1);
  std::string after_snapshot = marker(1) + std::string(9, 'Y') + "\n" + std::string(29, 'Z') + "\n";
  if (long_line) {
    // can't fit beside the snapshot: dropped, Y and Z too, so the gap stays at the start of the next upload
    log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(60, 'L'));
    CHECK(logs.unreported_drops() == 4);
    log(ESPHOME_LOG_LEVEL_INFO, "t", "after");
    after_snapshot = marker(4) + "after\n";
  }
  while (logs.upload_remaining() != 0) {
    n = logs.read(b, 33);
    snapshot.append(reinterpret_cast<char *>(b), n);
  }
  std::string expected;
  for (int i = 0; i < 8; i++)
    expected += std::string(30, 'a' + i) + "\n";
  CHECK(snapshot == expected);  // intact
  CHECK(upload(SIZE) == after_snapshot);
}

static void test_upload_snapshot_is_not_evicted() {
  upload_snapshot_is_not_evicted(false);
  upload_snapshot_is_not_evicted(true);
}

static void test_long_line_truncated() {
  drain();
  log(ESPHOME_LOG_LEVEL_INFO, "t", "evicted by the long line");
  log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(1000, 'z'));
  CHECK(upload(SIZE) == marker(1) + std::string(SIZE - 1, 'z') + "\n");
}

static void test_level_writes() {
  drain();
  log(ESPHOME_LOG_LEVEL_INFO, "t", "keep");
  CHECK(logs.set_level(ESPHOME_LOG_LEVEL_INFO, now_ms));  // the buffer is kept
  CHECK(logs.level() == ESPHOME_LOG_LEVEL_INFO);
  log(ESPHOME_LOG_LEVEL_DEBUG, "t", "below the host's level");
  CHECK(logs.set_level(0, now_ms));  // back to the default (DEBUG), still recording
  CHECK(logs.level() == ESPHOME_LOG_LEVEL_DEBUG);
  log(ESPHOME_LOG_LEVEL_DEBUG, "t", "default");
  CHECK(!logs.set_level(ESPHOME_LOG_LEVEL_VERY_VERBOSE + 1, now_ms));
  CHECK(logs.level() == ESPHOME_LOG_LEVEL_DEBUG);
  CHECK(upload(SIZE) == "keep\ndefault\n");
}

static void test_default_level_none() {
  uint8_t buf[64];
  LogBuffer off(buf, sizeof(buf));
  off.set_timeout(1000);
  off.start(0, 0);
  off.on_log(ESPHOME_LOG_LEVEL_ERROR, "t", "not recorded", 12);
  CHECK(off.pending() == 0 && !off.needs_timeout_check());
  CHECK(off.set_level(ESPHOME_LOG_LEVEL_INFO, 0));
  off.on_log(ESPHOME_LOG_LEVEL_INFO, "t", "recorded", 8);
  CHECK(off.needs_timeout_check());
  CHECK(off.check_timeout(1001) && off.level() == 0);  // no poll: back to off, the buffer is kept
  off.on_log(ESPHOME_LOG_LEVEL_ERROR, "t", "not recorded", 12);
  CHECK(off.pending() == 9);
  CHECK(off.set_level(0, 2000) && off.level() == 0);
}

static void test_upload_of_empty_buffer_has_size_0() {
  drain();
  CHECK(logs.begin_upload() == 0);  // the SDO server aborts a zero-length upload
}

static void test_timeout() {
  drain();
  logs.set_timeout(1000);
  CHECK(logs.set_level(ESPHOME_LOG_LEVEL_DEBUG, now_ms));  // the default: nothing to time out
  CHECK(!logs.needs_timeout_check());
  CHECK(logs.set_level(ESPHOME_LOG_LEVEL_VERBOSE, now_ms));
  CHECK(logs.needs_timeout_check());
  uint32_t t0 = now_ms;
  CHECK(!logs.check_timeout(t0 + 900));
  logs.keepalive(t0 + 900);  // a Log Pending read or a Log Data upload
  CHECK(!logs.check_timeout(t0 + 1800));
  log(ESPHOME_LOG_LEVEL_INFO, "t", "kept");
  CHECK(logs.check_timeout(t0 + 1901));
  CHECK(logs.level() == ESPHOME_LOG_LEVEL_DEBUG && !logs.needs_timeout_check());
  CHECK(!logs.check_timeout(t0 + 5000));  // already at the default
  log(ESPHOME_LOG_LEVEL_VERBOSE, "t", "above the default");
  CHECK(upload(SIZE) == "kept\n");

  // millis() wrap
  now_ms = 0xFFFFFF00u;
  CHECK(logs.set_level(ESPHOME_LOG_LEVEL_INFO, now_ms));
  CHECK(!logs.check_timeout(0x100));  // 512 ms later
  CHECK(logs.check_timeout(0x400));

  logs.set_timeout(0);  // never
  CHECK(logs.set_level(ESPHOME_LOG_LEVEL_INFO, now_ms));
  CHECK(!logs.needs_timeout_check());
  CHECK(!logs.check_timeout(now_ms + 0x7FFFFFFF));
  CHECK(logs.set_level(0, now_ms));
}

static void test_abandoned_upload() {
  drain();
  logs.set_timeout(1000);
  for (int i = 0; i < 10; i++)
    log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(30, 'a' + i));  // 2 evicted
  uint32_t t0 = now_ms;
  logs.keepalive(t0);
  uint8_t b[SIZE];
  CHECK(logs.begin_upload() == marker(2).size() + 8 * 31);
  CHECK(logs.read(b, 10) == 10);  // the host goes away inside the marker
  CHECK(logs.needs_timeout_check());  // the loop watches the upload
  CHECK(!logs.check_timeout(t0 + 1001));  // the level didn't change...
  CHECK(!logs.upload_in_progress());  // ...but the upload is given up
  // the snapshot is evictable again, the marker is reported again
  log(ESPHOME_LOG_LEVEL_INFO, "t", std::string(30, 'k'));
  std::string expected = marker(3);
  for (int i = 3; i < 10; i++)
    expected += std::string(30, 'a' + i) + "\n";
  CHECK(upload(SIZE) == expected + std::string(30, 'k') + "\n");
  logs.set_timeout(0);
}

// random lines, uploads read in random chunks with lines logged between reads, some uploads abandoned: every line
// arrives whole and in order, and lines received + marker counts == lines logged
static void test_random() {
  drain();
  CHECK(logs.unreported_drops() == 0);
  logs.set_timeout(1000);
  uint32_t seed = 12345;
  auto rnd = [&seed](uint32_t n) {
    seed = seed * 1103515245u + 12345u;
    return (seed >> 16) % n;
  };
  uint32_t logged = 0, next_expected = 0, accounted = 0;
  std::string partial;
  auto receive = [&](const std::string &data) {
    partial += data;
    size_t nl;
    while ((nl = partial.find('\n')) != std::string::npos) {
      std::string line = partial.substr(0, nl);
      partial.erase(0, nl + 1);
      if (line.rfind("[W][canopen.logs]: ", 0) == 0) {
        uint32_t n = std::stoul(line.substr(19));
        next_expected += n;
        accounted += n;
        continue;
      }
      size_t sp = line.find(' ');
      CHECK(sp != std::string::npos);
      uint32_t id = std::stoul(line.substr(1, sp - 1));
      CHECK(line == "#" + std::to_string(id) + " " + std::string(id % 41, char('a' + id % 26)));
      CHECK(id == next_expected);  // in order, nothing skipped without a marker
      next_expected++;
      accounted++;
    }
  };
  auto log_one = [&]() {
    uint32_t id = logged++;
    log(ESPHOME_LOG_LEVEL_INFO, "t", "#" + std::to_string(id) + " " + std::string(id % 41, char('a' + id % 26)));
  };
  for (int round = 0; round < 3000; round++) {
    for (uint32_t i = rnd(12); i != 0; i--)
      log_one();
    if (rnd(3) == 0)
      continue;
    uint32_t size = logs.begin_upload();
    if (size == 0)
      continue;
    // abandoned uploads stop after a whole line or inside the drop marker (sent again): a line cut in the middle
    // is lost by design (an aborted upload loses the bytes sent), the test couldn't attribute its rest
    bool abandon = rnd(8) == 0;
    uint32_t target = abandon ? rnd(size) : size;
    std::string got;
    auto may_stop = [&]() {
      if (got.size() >= size)
        return true;
      if (got.size() < target)
        return false;
      bool in_marker = got.rfind("[W][canopen.logs]", 0) == 0 && got.find('\n') == std::string::npos;
      return got.back() == '\n' || in_marker;
    };
    while (got.empty() || !may_stop()) {
      uint8_t b[SIZE];
      size_t want = got.size() >= target ? 1 : std::min<size_t>(1 + rnd(SIZE), target - got.size());
      size_t n = logs.read(b, want);
      got.append(reinterpret_cast<char *>(b), n);
      for (uint32_t i = rnd(3); i != 0; i--)
        log_one();
    }
    receive(got);
    if (got.size() < size) {
      now_ms += 2000;
      logs.check_timeout(now_ms);
      CHECK(!logs.upload_in_progress());
      partial.clear();  // the host drops a partial marker
    }
    CHECK(partial.empty());
    logs.keepalive(now_ms);
  }
  while (logs.pending() != 0) {
    uint32_t size = logs.begin_upload();
    std::string got;
    while (got.size() < size) {
      uint8_t b[SIZE];
      size_t n = logs.read(b, SIZE);
      got.append(reinterpret_cast<char *>(b), n);
    }
    receive(got);
  }
  CHECK(accounted == logged);
  CHECK(partial.empty());
  logs.set_timeout(0);
}

int main() {
  logs.set_timeout(30000);
  logs.start(ESPHOME_LOG_LEVEL_DEBUG, now_ms);
  CHECK(logs.level() == ESPHOME_LOG_LEVEL_DEBUG);  // records from boot at the default level

  test_filters();
  test_lines_logged_during_upload_are_kept();
  test_read_asks_for_more_than_left();
  test_wrap_around();
  test_overflow_evicts_oldest();
  test_upload_snapshot_is_not_evicted();
  test_long_line_truncated();
  test_level_writes();
  test_default_level_none();
  test_upload_of_empty_buffer_has_size_0();
  test_timeout();
  test_abandoned_upload();
  test_random();
  printf("OK\n");
  return 0;
}

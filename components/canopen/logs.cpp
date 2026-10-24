#include "logs.h"

#ifdef USE_CANOPEN_LOGS

#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace canopen {

static const uint32_t LOG_BUFFER_SIZE = CANOPEN_LOG_BUFFER_SIZE;
static const char DROP_MARKER_PREFIX[] = "[W][canopen.logs]: ";
static const char DROP_MARKER_SUFFIX[] = " messages dropped";

CanopenLogs canopen_logs;

void CanopenLogs::setup() {
  level = default_level;
  last_poll_ms = millis();
}

void CanopenLogs::poll() { last_poll_ms = millis(); }

void CanopenLogs::clear() {
  tail = 0;
  pending = 0;
  xfer_remaining = 0;
  unreported_drops = 0;
}

bool CanopenLogs::check_timeout(uint32_t now_ms) {
  if (!level || !timeout_ms || now_ms - last_poll_ms <= timeout_ms)
    return false;
  level = 0;
  clear();
  return true;
}

void CanopenLogs::put_(const char *data, size_t len) {
  uint32_t head = (tail + pending) % LOG_BUFFER_SIZE;
  size_t first = LOG_BUFFER_SIZE - head;
  if (first > len)
    first = len;
  memcpy(buf + head, data, first);
  memcpy(buf, data + first, len - first);
  pending += len;
}

// formats "<prefix><n><suffix>\n" into out, returns its length
static size_t format_drop_marker(char *out, uint32_t n) {
  size_t pos = sizeof(DROP_MARKER_PREFIX) - 1;
  memcpy(out, DROP_MARKER_PREFIX, pos);
  char digits[10];
  size_t num = 0;
  do {
    digits[num++] = '0' + n % 10;
    n /= 10;
  } while (n);
  while (num)
    out[pos++] = digits[--num];
  memcpy(out + pos, DROP_MARKER_SUFFIX, sizeof(DROP_MARKER_SUFFIX) - 1);
  pos += sizeof(DROP_MARKER_SUFFIX) - 1;
  out[pos++] = '\n';
  return pos;
}

void CanopenLogs::on_log(uint8_t msg_level, const char *tag, const char *message, size_t len) {
  if (!level || msg_level > level)
    return;
  // canbus logs every frame at DEBUG, and anything logged on the SDO path while serving a log read would feed itself
  if (msg_level > ESPHOME_LOG_LEVEL_INFO && (strcmp(tag, "canbus") == 0 || strcmp(tag, "canopen") == 0))
    return;
  while (len && (message[len - 1] == '\n' || message[len - 1] == '\0'))
    len--;
  if (len > LOG_BUFFER_SIZE - 1)
    len = LOG_BUFFER_SIZE - 1;

  // drop the newest whole line when it doesn't fit, the gap is marked in the stream once there is room again
  char marker[sizeof(DROP_MARKER_PREFIX) + sizeof(DROP_MARKER_SUFFIX) + 10];
  size_t marker_len = unreported_drops ? format_drop_marker(marker, unreported_drops) : 0;
  if (marker_len + len + 1 > LOG_BUFFER_SIZE - pending) {
    dropped++;
    unreported_drops++;
    return;
  }
  if (marker_len) {
    put_(marker, marker_len);
    unreported_drops = 0;
  }
  put_(message, len);
  put_("\n", 1);
}

uint32_t LogDataSize(CO_OBJ *obj, CO_NODE *node, uint32_t width) {
  // snapshot here too: block uploads of <= 4 bytes skip Reset
  auto *logs = (CanopenLogs *) obj->Data;
  logs->xfer_remaining = logs->pending;
  return logs->xfer_remaining;
}

CO_ERR LogDataReset(CO_OBJ *obj, CO_NODE *node, uint32_t para) {
  auto *logs = (CanopenLogs *) obj->Data;
  logs->xfer_remaining = logs->pending;
  return CO_ERR_NONE;
}

CO_ERR LogDataRead(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
  // never consume more than announced: the last block upload passes SegNum * 7, not the remaining size
  auto *logs = (CanopenLogs *) obj->Data;
  uint32_t n = size < logs->xfer_remaining ? size : logs->xfer_remaining;
  uint8_t *out = (uint8_t *) buffer;
  uint32_t first = LOG_BUFFER_SIZE - logs->tail;
  if (first > n)
    first = n;
  memcpy(out, logs->buf + logs->tail, first);
  memcpy(out + first, logs->buf, n - first);
  memset(out + n, 0, size - n);
  logs->tail = (logs->tail + n) % LOG_BUFFER_SIZE;
  logs->pending -= n;
  logs->xfer_remaining -= n;
  logs->poll();
  return CO_ERR_NONE;
}

uint32_t LogPendingSize(CO_OBJ *obj, CO_NODE *node, uint32_t width) { return 4; }

CO_ERR LogPendingRead(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
  auto *logs = (CanopenLogs *) obj->Data;
  if (size != 4)
    return CO_ERR_BAD_ARG;
  memcpy(buffer, &logs->pending, 4);
  logs->poll();
  return CO_ERR_NONE;
}

uint32_t LogLevelSize(CO_OBJ *obj, CO_NODE *node, uint32_t width) { return 1; }

CO_ERR LogLevelRead(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
  auto *logs = (CanopenLogs *) obj->Data;
  if (size != 1)
    return CO_ERR_BAD_ARG;
  *(uint8_t *) buffer = logs->level;
  return CO_ERR_NONE;
}

CO_ERR LogLevelWrite(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
  auto *logs = (CanopenLogs *) obj->Data;
  if (size != 1)
    return CO_ERR_BAD_ARG;
  uint8_t value = *(uint8_t *) buffer;
  if (value > ESPHOME_LOG_LEVEL_VERY_VERBOSE)
    return CO_ERR_TYPE_WR;
  // rewriting the current level is the keepalive and keeps the buffer
  if (!logs->level && value)
    logs->clear();
  logs->level = value;
  logs->poll();
  return CO_ERR_NONE;
}

}  // namespace canopen
}  // namespace esphome
#endif

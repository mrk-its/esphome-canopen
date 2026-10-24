#pragma once

#include "esphome/core/defines.h"

#ifdef USE_CANOPEN_LOGS

#include <cstddef>
#include <cstdint>

#include "co_core.h"

namespace esphome {
namespace canopen {

// Remote logging over CANopen, OD 0x3100 (see OBJECT_DICTIONARY.md). Log lines are buffered in a ring buffer
// and consumed by the host with SDO uploads of 0x3100:1. Everything here runs on the main loop (log listeners
// and SDO processing both do), so there is no locking. None of these functions may log: a log call would feed
// back into on_log() in the middle of a consume.
struct CanopenLogs {
  uint8_t buf[CANOPEN_LOG_BUFFER_SIZE];
  uint16_t tail;            // read position
  uint32_t pending;         // buffered bytes, head = (tail + pending) % size
  uint32_t xfer_remaining;  // bytes announced by the current upload (snapshot in Size / Reset)
  uint32_t dropped;         // lines dropped because the buffer was full, since boot (0x3100:4)
  uint32_t unreported_drops;
  uint32_t timeout_ms;  // 0 = never
  uint32_t last_poll_ms;
  uint8_t level;  // 0 = off, 1..7 = ESPHOME_LOG_LEVEL_ERROR..VERY_VERBOSE
  uint8_t default_level;

  void setup();
  void on_log(uint8_t msg_level, const char *tag, const char *message, size_t len);
  void clear();
  // turns logging off when no host polled for timeout_ms, returns true if it did
  bool check_timeout(uint32_t now_ms);
  void poll();

 protected:
  void put_(const char *data, size_t len);
};

extern CanopenLogs canopen_logs;

uint32_t LogDataSize(CO_OBJ *obj, CO_NODE *node, uint32_t width);
CO_ERR LogDataRead(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size);
CO_ERR LogDataReset(CO_OBJ *obj, CO_NODE *node, uint32_t para);
const CO_OBJ_TYPE LogDataType = {LogDataSize, 0, LogDataRead, 0, LogDataReset};
#define LOGS_DATA ((CO_OBJ_TYPE *) &LogDataType)

uint32_t LogPendingSize(CO_OBJ *obj, CO_NODE *node, uint32_t width);
CO_ERR LogPendingRead(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size);
const CO_OBJ_TYPE LogPendingType = {LogPendingSize, 0, LogPendingRead, 0, 0};
#define LOGS_PENDING ((CO_OBJ_TYPE *) &LogPendingType)

uint32_t LogLevelSize(CO_OBJ *obj, CO_NODE *node, uint32_t width);
CO_ERR LogLevelRead(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size);
CO_ERR LogLevelWrite(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size);
const CO_OBJ_TYPE LogLevelType = {LogLevelSize, 0, LogLevelRead, LogLevelWrite, 0};
#define LOGS_LEVEL ((CO_OBJ_TYPE *) &LogLevelType)

}  // namespace canopen
}  // namespace esphome
#endif

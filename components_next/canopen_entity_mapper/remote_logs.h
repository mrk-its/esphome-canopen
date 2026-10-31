#pragma once

#include "esphome/core/defines.h"

#ifdef USE_CANOPEN_ENTITY_MAPPER_LOGS

#include <array>

#include "esphome/components/canopen/canopen.h"
#include "esphome/core/component.h"
#include "log_buffer.h"

namespace esphome::canopen_entity_mapper {

// Remote logs (`canopen_entity_mapper: remote_logs:`), OD 0x3100 as can2mqtt reads it (OBJECT_DICTIONARY.md,
// docs/remote-logs.md):
//   :1 Log Data (domain, an upload consumes it), :2 Log Pending, :3 Log Level (RW, the host's subscription),
//   :4 Log Dropped, :5 default level, :6 timeout (ms, 0 = never), :7 SDO block size for Log Data uploads
// The handlers must not log (see LogBuffer). loop() only runs while logging is on with a timeout.
class RemoteLogs : public Component, public canopen::OdObject {
 public:
  static constexpr uint16_t OD_INDEX = 0x3100;

  RemoteLogs() : buffer_(this->storage_.data(), this->storage_.size()) {}

  void set_default_level(uint8_t level) { this->default_level_ = level; }
  void set_timeout(uint32_t timeout_ms) { this->timeout_ms_ = timeout_ms; }
  void set_block_size(uint8_t block_size) { this->block_size_ = block_size; }
  // adds the OD record (from codegen, before the hub's setup)
  void set_canopen(canopen::CANopenComponent *hub);
  // registers the logger callback; from codegen before App.setup(), so the setup logs of all components are buffered
  void start();

  void setup() override;
  void loop() override;
  void dump_config() override;

  canopen::OdResult od_read(canopen::OdStream stream, void *buf, size_t count, size_t *done);
  canopen::OdResult od_write(canopen::OdStream stream, const void *buf, size_t count, size_t *done);

 protected:
  void update_loop_();

  std::array<uint8_t, CANOPEN_ENTITY_MAPPER_LOG_BUFFER_SIZE> storage_{};
  LogBuffer buffer_;

  // OD 0x3100 storage; :2 to :4 are copied from buffer_ when read
  uint8_t highest_{7};
  uint32_t pending_{0};
  uint8_t level_{0};
  uint32_t dropped_{0};
  uint8_t default_level_{0};
  uint32_t timeout_ms_{0};
  uint8_t block_size_{3};
};

}  // namespace esphome::canopen_entity_mapper

#endif  // USE_CANOPEN_ENTITY_MAPPER_LOGS

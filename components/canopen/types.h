#pragma once

// Public CANopen types for lambdas and other components. No CANopenNode headers here.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace esphome {
struct LogString;
}  // namespace esphome

namespace esphome::canopen {

// NMT state as sent in a heartbeat (CiA 301); also used for our own state
enum class NMTState : uint8_t {
  NMT_STATE_BOOTUP = 0,
  NMT_STATE_STOPPED = 4,
  NMT_STATE_OPERATIONAL = 5,
  NMT_STATE_PRE_OPERATIONAL = 127,
  NMT_STATE_UNKNOWN = 255,
};

const LogString *nmt_state_to_string(NMTState state);

// Events of a monitored node (CANopenComponent::monitor_node(), from CANopenNode's heartbeat consumer):
// ONLINE: first heartbeat, or the first after a timeout; OFFLINE: no heartbeat within the timeout;
// NMT_CHANGED: the state in the heartbeat changed (also unknown -> state right after ONLINE, state -> unknown after
// OFFLINE); RESET: boot-up message, the node restarted and lost its runtime configuration.
enum class NodeEvent : uint8_t { NODE_EVENT_ONLINE, NODE_EVENT_OFFLINE, NODE_EVENT_NMT_CHANGED, NODE_EVENT_RESET };

const LogString *node_event_to_string(NodeEvent event);

// NMT commands (CiA 301), see CANopenComponent::send_nmt()
enum class NMTCommand : uint8_t {
  NMT_COMMAND_START = 1,
  NMT_COMMAND_STOP = 2,
  NMT_COMMAND_PRE_OPERATIONAL = 128,
  NMT_COMMAND_RESET_NODE = 129,
  NMT_COMMAND_RESET_COMMUNICATION = 130,
};

// one object mapped into a PDO, see CANopenComponent::configure_tpdo() / configure_rpdo()
struct PdoMapping {
  uint16_t index;
  uint8_t sub;
  uint8_t bits;  // multiple of 8
};

enum class SDOResult : uint8_t {
  SDO_RESULT_SUCCESS,
  SDO_RESULT_ABORT,            // the server aborted the transfer, see abort_code
  SDO_RESULT_TIMEOUT,          // no response within response_timeout
  SDO_RESULT_SIZE_MISMATCH,    // a value read returned another width than expected
  SDO_RESULT_TRANSPORT_ERROR,  // the transfer couldn't be started or failed locally
  SDO_RESULT_REJECTED,         // not queued (queue full, invalid arguments, stack not running)
};

const LogString *sdo_result_to_string(SDOResult result);

// Result of an SDO request. Value reads/writes use value/size (little endian, zero-extended); buffer reads use
// data/len, valid only during the callback. abort_code is set for ABORT and TIMEOUT.
struct SDOReply {
  SDOResult result;
  uint32_t abort_code;
  uint32_t value;
  uint8_t size;
  const uint8_t *data;
  size_t len;

  bool ok() const { return this->result == SDOResult::SDO_RESULT_SUCCESS; }
  // value as T: sign-extended from size bytes for signed T, bit cast for float
  template<typename T> T as() const {
    static_assert(sizeof(T) <= 4, "SDO values are at most 4 bytes");
    if constexpr (std::is_same_v<T, float>) {
      float f;
      std::memcpy(&f, &this->value, sizeof(f));
      return f;
    } else if constexpr (std::is_signed_v<T>) {
      if (this->size == 0 || this->size >= 4)
        return static_cast<T>(static_cast<int32_t>(this->value));
      uint32_t sign = 1u << (this->size * 8 - 1);
      return static_cast<T>(static_cast<int32_t>((this->value ^ sign) - sign));
    } else {
      return static_cast<T>(this->value);
    }
  }
};

// Called exactly once per request: from CANopenComponent::loop() / frame processing, or synchronously (REJECTED)
// before read_sdo() / write_sdo() return false.
using SDOCallback = void (*)(void *context, const SDOReply &reply);

// raw CAN frame listener (see CANopenComponent::add_frame_listener()); data is valid only during the call
using FrameCallback = void (*)(void *context, uint32_t can_id, const uint8_t *data, uint8_t len);

// EMCY frame (CiA 301): error code, error register (0x1001), manufacturer-specific bytes
struct Emergency {
  uint16_t error_code;
  uint8_t error_register;
  std::array<uint8_t, 5> manufacturer_data;
};

// Heartbeat frame: COB-ID 0x700 + node (1..127), 1 byte. Returns false for any other frame.
inline bool decode_heartbeat(uint32_t can_id, const uint8_t *data, size_t len, uint8_t &node, NMTState &state) {
  if (can_id < 0x701 || can_id > 0x77F || len != 1)
    return false;
  node = can_id - 0x700;
  // bit 7 is the node guarding toggle bit, always 0 in a heartbeat
  switch (data[0] & 0x7F) {
    case 0:
      state = NMTState::NMT_STATE_BOOTUP;
      break;
    case 4:
      state = NMTState::NMT_STATE_STOPPED;
      break;
    case 5:
      state = NMTState::NMT_STATE_OPERATIONAL;
      break;
    case 127:
      state = NMTState::NMT_STATE_PRE_OPERATIONAL;
      break;
    default:
      state = NMTState::NMT_STATE_UNKNOWN;
      break;
  }
  return true;
}

// EMCY frame: COB-ID 0x80 + node (1..127; 0x80 itself is SYNC), 8 bytes. Returns false for any other frame.
inline bool decode_emcy(uint32_t can_id, const uint8_t *data, size_t len, uint8_t &node, Emergency &emcy) {
  if (can_id < 0x81 || can_id > 0xFF || len != 8)
    return false;
  node = can_id - 0x80;
  emcy.error_code = data[0] | (data[1] << 8);
  emcy.error_register = data[2];
  for (size_t i = 0; i < emcy.manufacturer_data.size(); i++)
    emcy.manufacturer_data[i] = data[3 + i];
  return true;
}

}  // namespace esphome::canopen

#pragma once

// Public CANopen types for lambdas and other components. No CANopenNode headers here.

#include <array>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace canopen {

// NMT state as sent in a heartbeat (CiA 301); also used for our own state
enum class NMTState : uint8_t {
  BOOTUP = 0,
  STOPPED = 4,
  OPERATIONAL = 5,
  PRE_OPERATIONAL = 127,
  UNKNOWN = 255,
};

const char *nmt_state_to_string(NMTState state);

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
      state = NMTState::BOOTUP;
      break;
    case 4:
      state = NMTState::STOPPED;
      break;
    case 5:
      state = NMTState::OPERATIONAL;
      break;
    case 127:
      state = NMTState::PRE_OPERATIONAL;
      break;
    default:
      state = NMTState::UNKNOWN;
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

}  // namespace canopen
}  // namespace esphome

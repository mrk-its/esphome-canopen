#pragma once

#include <cstdint>

namespace esphome::canopen_entities {

// A node-to-node OD write received on 0x500..0x57F (OBJECT_DICTIONARY.md "OD writer"). Two formats, told apart by
// bit 7 of byte 0, which is always clear in the old one because node ids end at 127:
//   DAM-MPDO (CiA 301): 0x80 | target, index (2 bytes, little endian), sub, value padded to 4 bytes, always 8 bytes
//   old OD writer:      target, sub, index (2 bytes, little endian), value of 1..4 bytes, 5..8 bytes
struct OdWriterFrame {
  uint16_t index;
  uint8_t sub;
  const uint8_t *value;
  uint8_t len;
  bool dam;  // value is padded: the target object decides how many bytes count
};

// False if the frame is malformed or for another node. A DAM-MPDO to address 0 (all nodes) is ignored too: the same
// index is a different entity on each node.
inline bool decode_od_writer_frame(const uint8_t *data, uint8_t len, uint8_t node_id, OdWriterFrame &frame) {
  if (len < 5 || len > 8)
    return false;
  frame.dam = (data[0] & 0x80) != 0;
  if ((data[0] & 0x7F) != node_id || (frame.dam && len != 8))
    return false;
  if (frame.dam) {
    frame.index = data[1] | (data[2] << 8);
    frame.sub = data[3];
  } else {
    frame.sub = data[1];
    frame.index = data[2] | (data[3] << 8);
  }
  frame.value = data + 4;
  frame.len = len - 4;
  return true;
}

// If index is an entity's command object (0x2002 + 16 * n), gives its state object (0x2001 + 16 * n), which has the same
// subs. Anything else written through the OD writer isn't a command.
inline bool od_writer_state_index(uint16_t index, uint16_t &state_index) {
  if (index < 0x2000 || index >= 0x3000 || (index & 0xF) != 2)
    return false;
  state_index = index - 1;
  return true;
}

// The old format: all 8 bytes are filled, the frame is 4 + size bytes long.
inline void encode_old_od_writer_frame(uint8_t *data, uint8_t target, uint16_t index, uint8_t sub, uint32_t value) {
  data[0] = target;
  data[1] = sub;
  data[2] = uint8_t(index);
  data[3] = uint8_t(index >> 8);
  for (uint8_t i = 0; i < 4; i++)
    data[4 + i] = uint8_t(value >> (8 * i));
}

// A DAM-MPDO, always 8 bytes: the value's size bytes (1..4), padded with zeros.
inline void encode_dam_mpdo_frame(uint8_t *data, uint8_t target, uint16_t index, uint8_t sub, uint32_t value,
                                  uint8_t size) {
  data[0] = 0x80 | target;
  data[1] = uint8_t(index);
  data[2] = uint8_t(index >> 8);
  data[3] = sub;
  for (uint8_t i = 0; i < 4; i++)
    data[4 + i] = i < size ? uint8_t(value >> (8 * i)) : 0;
}

}  // namespace esphome::canopen_entities

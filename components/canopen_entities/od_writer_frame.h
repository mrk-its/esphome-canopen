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

}  // namespace esphome::canopen_entities

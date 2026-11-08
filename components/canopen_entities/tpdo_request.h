#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::canopen_entities {

// Targeted TPDO requests (OBJECT_DICTIONARY.md "TPDO request"): the TPDOs this node listens to on one remote node.
struct TpdoRequest {
  uint8_t node_id;
  uint8_t mask;  // bit n: TPDO n
  bool pending;
};

// Nodes that boot together must not all ask at once, so the delay depends on the node id, 50..500 ms. At least 50 ms:
// a node that just sent its boot-up message may not be OPERATIONAL yet, and it ignores a DAM-MPDO until it is.
inline uint16_t tpdo_request_delay_ms(uint8_t node_id) { return 50 + (node_id * 37) % 451; }

// Marks the request to node_id as pending, however often it's called. False if this node doesn't watch node_id.
inline bool mark_tpdo_request(TpdoRequest *requests, size_t count, uint8_t node_id) {
  for (size_t i = 0; i < count; i++) {
    if (requests[i].node_id == node_id) {
      requests[i].pending = true;
      return true;
    }
  }
  return false;
}

}  // namespace esphome::canopen_entities

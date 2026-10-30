#pragma once

// can2mqtt discovery test: one fake sensor in the esphome-canopen entity layout (OBJECT_DICTIONARY.md), built on the
// hub's OD API, with its TPDO emulated by send_frame() (the hub has no own PDOs yet). Needs the ESPHome identity
// (vendor 0xA59A08F5, product 0x6BDFA1D9, revision 1) in the canopen config. register_objects() runs from on_boot,
// before the hub's setup().

#include <cmath>
#include <cstring>

#include "esphome/components/canopen/canopen.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace can2mqtt_test {

using namespace esphome::canopen;
static const char *const TAG = "can2mqtt_test";

constexpr uint8_t ENTITY = 1;                            // entity index N
constexpr uint16_t META = 0x2000 + ENTITY * 0x10;        // metadata
constexpr uint16_t STATES = 0x2001 + ENTITY * 0x10;      // states
constexpr uint32_t SENSOR_TYPE = 1;                      // EntityTypeCode sensor, version 0, no caps

struct Storage {
  // 0x2001: entity types, sub N = type of entity N
  uint8_t types_count{1};
  uint32_t types[1]{SENSOR_TYPE};
  // metadata (strings)
  uint8_t meta_highest{4};
  // states: sub1 = REAL32 as raw bits
  uint8_t states_highest{1};
  float state{0};
  // TPDO 0..7 communication / mapping parameters (read by can2mqtt; only TPDO 0 is valid)
  uint8_t comm_highest{2};
  uint32_t cob_id[8]{};
  uint8_t trans_type[8]{};
  uint8_t map_count[8]{};
  uint32_t map0{(uint32_t(STATES) << 16) | (1 << 8) | 32};  // 0x2011:1, 32 bits
};
static Storage s;
static const char NAME[] = "CANopenNode test sensor";
static const char UNIT[] = "s";

inline void register_objects(CANopenComponent *co, uint8_t node_id) {
  auto &od = co->get_od();
  od.add_array(0x2001, OD_READ, &s.types_count, s.types, sizeof(uint32_t), 1);
  od.add_record(META, {{0, OD_READ, &s.meta_highest, 1},
                       {1, OD_READ, (void *) NAME, sizeof(NAME) - 1},
                       {3, OD_READ, (void *) UNIT, sizeof(UNIT) - 1}});
  od.add_record(STATES, {{0, OD_READ, &s.states_highest, 1}, {1, OD_READ | OD_TPDO, &s.state, 4}});
  for (int n = 0; n < 8; n++) {
    s.cob_id[n] = n == 0 ? 0x180u + node_id : 0x80000000u;
    s.trans_type[n] = 254;
    s.map_count[n] = n == 0 ? 1 : 0;
    od.add_record(0x1800 + n, {{0, OD_READ, &s.comm_highest, 1},
                               {1, OD_READ, &s.cob_id[n], 4},
                               {2, OD_READ, &s.trans_type[n], 1}});
    if (n == 0) {
      od.add_record(0x1A00, {{0, OD_READ, &s.map_count[0], 1}, {1, OD_READ, &s.map0, 4}});
    } else {
      od.add_record(0x1A00 + n, {{0, OD_READ, &s.map_count[n], 1}});
    }
  }
}

// new value (uptime in seconds), sent as TPDO 0
inline void update(CANopenComponent *co, uint8_t node_id) {
  s.state = std::round(esphome::millis() / 100.0f) / 10.0f;
  uint8_t data[4];
  std::memcpy(data, &s.state, 4);
  bool sent = co->send_frame(0x180 + node_id, data, sizeof(data));
  ESP_LOGD(TAG, "state %.1f, TPDO 0x%03x %s", s.state, 0x180 + node_id, sent ? "sent" : "refused");
}

}  // namespace can2mqtt_test

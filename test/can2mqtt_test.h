#pragma once

// can2mqtt discovery test: one fake sensor in the esphome-canopen entity layout (OBJECT_DICTIONARY.md), built on the
// hub's OD API, with its state sent as a real TPDO (PR 9). Also an RPDO of another node's TPDO. Needs the ESPHome
// identity (vendor 0xA59A08F5, product 0x6BDFA1D9, revision 1) in the canopen config and canopen.enable_pdo() (here
// from canopen_test_profile). register_objects() runs from on_boot, before the hub's setup().

#include <cmath>

#include "esphome/components/canopen/canopen.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace can2mqtt_test {

using namespace esphome::canopen;
static const char *const TAG = "can2mqtt_test";

constexpr uint8_t ENTITY = 1;                        // entity index N
constexpr uint16_t META = 0x2000 + ENTITY * 0x10;    // metadata
constexpr uint16_t STATES = 0x2001 + ENTITY * 0x10;  // states
constexpr uint32_t SENSOR_TYPE = 1;                  // EntityTypeCode sensor, version 0, no caps
constexpr uint16_t RX = 0x5002;                      // RPDO target: node 0x63's TPDO 0 (6 bytes)

// the sensor's state: a handler, so request_tpdo(STATES, 1) has its request flag
struct State : OdObject {
  float value{0};
};

// RPDO target: logs what arrives
struct Received : OdObject {
  uint8_t data[6]{};
  int frames{0};
  OdResult od_write(OdStream stream, const void *buf, size_t count, size_t *done) {
    OdResult ret = stream.write_original(buf, count, done);
    if (++this->frames <= 3 || this->frames % 10 == 0) {
      ESP_LOGI(TAG, "RPDO 0x1e3 #%d: %02x %02x %02x %02x %02x %02x", this->frames, this->data[0], this->data[1],
               this->data[2], this->data[3], this->data[4], this->data[5]);
    }
    return ret;
  }
};

struct Storage {
  uint8_t types_count{1};
  uint32_t types[1]{SENSOR_TYPE};
  uint8_t meta_highest{4};
  uint8_t states_highest{1};
};
static Storage s;
static State state;
static Received received;
static const char NAME[] = "CANopenNode test sensor";
static const char UNIT[] = "s";

inline void register_objects(CANopenComponent *co, uint8_t node_id) {
  auto &od = co->get_od();
  od.add_array(0x2001, OD_READ, &s.types_count, s.types, sizeof(uint32_t), 1);
  od.add_record(META, {{0, OD_READ, &s.meta_highest, 1},
                       {1, OD_READ, (void *) NAME, sizeof(NAME) - 1},
                       {3, OD_READ, (void *) UNIT, sizeof(UNIT) - 1}});
  od.add_record(STATES, {{0, OD_READ, &s.states_highest, 1}, {1, OD_READ | OD_TPDO, &state.value, 4}});
  od.set_object(STATES, &state);
  od.add_var(RX, OD_READ_WRITE | OD_RPDO, received.data, sizeof(received.data));
  od.set_object(RX, &received);
  // can2mqtt reads TPDOs 0..7, so all 8 exist: TPDO 0 maps the state, TPDO 7 (no mappings) is disabled and the
  // ones between are placeholders
  co->configure_tpdo(0, 0x180 + node_id, {{STATES, 1, 32}});
  co->configure_tpdo(7, 0x500 + node_id, {});
  // RPDO 0 receives node 0x63's TPDO 0
  co->configure_rpdo(0, 0x1E3, {{RX, 0, 48}});
}

// new value (uptime in seconds), sent as TPDO 0 by CANopenNode
inline void update(CANopenComponent *co) {
  state.value = std::round(esphome::millis() / 100.0f) / 10.0f;
  bool requested = co->request_tpdo(STATES, 1);
  ESP_LOGD(TAG, "state %.1f, TPDO requested: %s", state.value, requested ? "yes" : "no");
}

}  // namespace can2mqtt_test

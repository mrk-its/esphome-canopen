#pragma once

#include <memory>
#include <string>
#include <vector>

#include "esphome/components/canopen/canopen.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

namespace esphome {
namespace esphome_entity_mapper {

// Entity type codes (0x2001:N bits 0..7), see OBJECT_DICTIONARY.md
enum EntityType : uint8_t {
  ENTITY_TYPE_SENSOR = 1,
  ENTITY_TYPE_BINARY_SENSOR = 2,
  ENTITY_TYPE_SENSOR_UINT8 = 6,
  ENTITY_TYPE_SENSOR_UINT16 = 7,
};

// wire encoding of scaled states, see "Scaled values" in OBJECT_DICTIONARY.md
uint32_t scale_to_wire(float value, float min_val, float max_val, uint32_t max_int);

// One exposed entity: metadata at 0x2000 + 16 * index, states at 0x2001 + 16 * index (the handler gives the states
// their TPDO request flags).
class MappedEntity : public canopen::OdObject {
 public:
  MappedEntity(uint8_t index, uint32_t type) : index_(index), type_(type) {}

  uint16_t meta_index() const { return 0x2000 + 0x10 * this->index_; }
  uint16_t states_index() const { return 0x2001 + 0x10 * this->index_; }
  uint8_t get_index() const { return this->index_; }
  uint32_t get_type() const { return this->type_; }
  uint32_t *type_storage() { return &this->type_; }

  // states in subindex order (layout.py entity_states()), all before add_to_od()
  void add_state(uint8_t size, uint32_t value) {
    this->state_sizes_.push_back(size);
    this->states_.push_back(value);
  }
  void set_metadata(std::string name, std::string device_class, std::string unit, std::string state_class) {
    this->name_ = std::move(name);
    this->device_class_ = std::move(device_class);
    this->unit_ = std::move(unit);
    this->state_class_ = std::move(state_class);
  }
  void set_min_max(float min_value, float max_value);

  void add_to_od(canopen::ObjectDictionary &od);
  // new value of state sub (1..), sent by the TPDO that maps it
  void set_state(canopen::CANopenComponent *hub, uint8_t sub, uint32_t value);

 protected:
  uint8_t index_;
  uint32_t type_;
  std::vector<uint8_t> state_sizes_;
  std::vector<uint32_t> states_;  // little-endian storage, size bytes used
  uint8_t states_count_{0};
  uint8_t meta_highest_{4};
  std::string name_, device_class_, unit_, state_class_;
  bool has_min_max_{false};
  uint32_t min_value_{0}, max_value_{0};  // REAL32 bits
};

class EntityMapper : public Component {
 public:
  explicit EntityMapper(canopen::CANopenComponent *hub) : hub_(hub) {}

  void set_layout_hash(uint32_t hash) { this->layout_hash_ = hash; }
#ifdef USE_SENSOR
  void add_sensor(sensor::Sensor *sensor, uint8_t index, uint8_t size, float min_value, float max_value);
#endif
#ifdef USE_BINARY_SENSOR
  void add_binary_sensor(binary_sensor::BinarySensor *sensor, uint8_t index);
#endif

  // adds the OD entries: before the hub's setup()
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR + 1.0f; }

 protected:
  MappedEntity *add_entity_(uint8_t index, uint32_t type);

  canopen::CANopenComponent *hub_;
  std::vector<std::unique_ptr<MappedEntity>> entities_;
  // 0x2001: entity types, sub N = type of entity N
  uint8_t types_highest_{0};
  // 0x3002: TPDO layout hash (layout.py)
  uint8_t layout_highest_{1};
  uint32_t layout_hash_{0};
};

}  // namespace esphome_entity_mapper
}  // namespace esphome

#pragma once

#include <array>
#include <span>

#include "esphome/components/canopen/canopen.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/entity_base.h"
#include "esphome/core/helpers.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_LIGHT
#include "esphome/components/light/light_state.h"
#endif
#ifdef USE_COVER
#include "esphome/components/cover/cover.h"
#endif
#ifdef USE_ALARM_CONTROL_PANEL
#include "esphome/components/alarm_control_panel/alarm_control_panel.h"
#endif

namespace esphome::canopen_entity_mapper {

// Entity type codes (0x2001:N bits 0..7), see OBJECT_DICTIONARY.md
enum EntityType : uint8_t {
  ENTITY_TYPE_SENSOR = 1,
  ENTITY_TYPE_BINARY_SENSOR = 2,
  ENTITY_TYPE_SWITCH = 3,
  ENTITY_TYPE_COVER = 4,
  ENTITY_TYPE_LIGHT = 5,
  ENTITY_TYPE_SENSOR_UINT8 = 6,
  ENTITY_TYPE_SENSOR_UINT16 = 7,
  ENTITY_TYPE_NUMBER = 8,
  ENTITY_TYPE_NUMBER_UINT8 = 9,
  ENTITY_TYPE_NUMBER_UINT16 = 10,
  ENTITY_TYPE_ALARM = 16,
};

// light / cover capability bits (0x2001:N bits 16..23), from codegen (layout.py)
enum : uint32_t {
  LIGHT_CAP_ON_OFF = 1,
  LIGHT_CAP_BRIGHTNESS = 2,
  LIGHT_CAP_COLOR_TEMPERATURE = 4,
  COVER_CAP_POSITION = 1,
  COVER_CAP_TILT = 2,
};

// wire encoding of scaled states and commands, see "Scaled values" in OBJECT_DICTIONARY.md (max_int = NaN)
uint32_t scale_to_wire(float value, float min_val, float max_val, uint32_t max_int);
float scale_from_wire(uint32_t wire, float min_val, float max_val, uint32_t max_int);
inline uint32_t percentage_to_wire(float value) { return scale_to_wire(value, 0.0f, 1.0f, 255); }
inline float percentage_from_wire(uint32_t wire) { return scale_from_wire(wire, 0.0f, 1.0f, 255); }
inline uint32_t color_temp_to_wire(float mireds) { return scale_to_wire(mireds, 100.0f, 1000.0f, 255); }
inline float color_temp_from_wire(uint32_t wire) { return scale_from_wire(wire, 100.0f, 1000.0f, 255); }

// One exposed entity: metadata at 0x2000 + 16 * index, states at 0x2001 + 16 * index, commands at 0x2002 + 16 * index
// (one per state, same size; write only, RPDO mappable). As the handler of the three records it reads the metadata
// strings from the entity on each upload (no copies), gives the states their TPDO request flags and runs the commands.
// Callbacks capture only the MappedEntity pointer.
class MappedEntity : public canopen::OdObject {
 public:
  static constexpr size_t MAX_STATES = 3;

  void init(canopen::CANopenComponent *hub, EntityBase *entity, uint8_t index, uint32_t type);

  uint16_t meta_index() const { return 0x2000 + 0x10 * this->index_; }
  uint16_t states_index() const { return 0x2001 + 0x10 * this->index_; }
  uint16_t commands_index() const { return 0x2002 + 0x10 * this->index_; }
  uint8_t get_index() const { return this->index_; }
  uint32_t get_type() const { return this->type_; }
  uint32_t *type_storage() { return &this->type_; }
  // light / cover capabilities (type bits 16..23)
  uint32_t get_caps() const { return (this->type_ >> 16) & 0xFF; }

  // states in subindex order (layout.py entity_states()), all before add_to_od()
  void add_state(uint8_t size, uint32_t value);
  // metadata subindices 3 (unit) and 4 (state class): sensors only
  void set_sensor_metadata(bool unit, uint8_t state_class) {
    this->has_unit_ = unit;
    this->state_class_ = state_class;
  }
  // scale of a 1 / 2 byte sensor or number (metadata subindices 7 / 8), also the color temperature range of a light
  void set_min_max(float min_value, float max_value);

  void add_to_od(canopen::ObjectDictionary &od);
  // new value of state sub (1..), sent by the TPDO that maps it
  void set_state(uint8_t sub, uint32_t value);
  // sensor / number value, scaled to the state's size
  void set_value(float value) { this->set_state(1, this->value_to_wire(value)); }
  uint32_t value_to_wire(float value) const;
  float value_from_wire(uint32_t wire) const;
#ifdef USE_COVER
  void update_cover();
#endif
#ifdef USE_LIGHT
  void update_light();
#endif

  canopen::OdResult od_read(canopen::OdStream stream, void *buf, size_t count, size_t *done);
  canopen::OdResult od_write(canopen::OdStream stream, const void *buf, size_t count, size_t *done);

 protected:
  // the metadata string of sub into buf (null terminated), its length; 0 = none
  size_t meta_string_(uint8_t sub, std::span<char, ENTITY_NAME_BUF_SIZE> buf) const;
  // command sub (1..) with its value (size bytes of the state it controls, little endian)
  void command_(uint8_t sub, uint32_t value);
#ifdef USE_LIGHT
  void light_command_(uint8_t sub, uint32_t value);
#endif
#ifdef USE_COVER
  void cover_command_(uint8_t sub, uint32_t value);
#endif
#ifdef USE_ALARM_CONTROL_PANEL
  void alarm_command_(uint32_t value);
#endif

  canopen::CANopenComponent *hub_{nullptr};
  EntityBase *entity_{nullptr};
  uint32_t type_{0};
  std::array<uint32_t, MAX_STATES> states_{};  // little-endian storage, size bytes used
  std::array<uint8_t, MAX_STATES> state_sizes_{};
  std::array<uint32_t, MAX_STATES> commands_{};  // the last written commands (storage for SDO and RPDO writes)
  uint8_t states_count_{0};
  uint8_t index_{0};
  uint8_t meta_highest_{4};
  uint8_t state_class_{0};  // sensor::StateClass, 0 = none
  bool has_unit_{false};
  bool has_min_max_{false};
  float min_value_{0.0f};  // REAL32 in the OD
  float max_value_{0.0f};
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
#ifdef USE_NUMBER
  void add_number(number::Number *number, uint8_t index, uint8_t size, float min_value, float max_value);
#endif
#ifdef USE_SWITCH
  void add_switch(switch_::Switch *sw, uint8_t index);
#endif
#ifdef USE_LIGHT
  void add_light(light::LightState *light, uint8_t index, uint32_t caps);
#endif
#ifdef USE_COVER
  void add_cover(cover::Cover *cover, uint8_t index, uint32_t caps);
#endif
#ifdef USE_ALARM_CONTROL_PANEL
  void add_alarm(alarm_control_panel::AlarmControlPanel *alarm, uint8_t index);
#endif

  // adds the OD entries: before the hub's setup()
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR + 1.0f; }

 protected:
  MappedEntity *add_entity_(EntityBase *entity, uint8_t index, uint32_t type);
#ifdef USE_LIGHT
  // light values reach the mapper through a listener object
  class LightListener : public light::LightRemoteValuesListener {
   public:
    explicit LightListener(MappedEntity *entity) : entity_(entity) {}
    void on_light_remote_values_update() override { this->entity_->update_light(); }

   protected:
    MappedEntity *entity_;
  };
#endif

  canopen::CANopenComponent *hub_;
  // CANOPEN_ENTITY_MAPPER_ENTITIES: from codegen, the number of entities
  StaticVector<MappedEntity, CANOPEN_ENTITY_MAPPER_ENTITIES> entities_;
  // 0x2001: entity types, sub N = type of entity N
  uint8_t types_highest_{0};
  // 0x3002: TPDO layout hash (layout.py)
  uint8_t layout_highest_{1};
  uint32_t layout_hash_{0};
};

}  // namespace esphome::canopen_entity_mapper

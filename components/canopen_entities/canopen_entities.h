#pragma once

#include <array>
#include <span>

#include "esphome/components/canopen/canopen.h"
#include "esphome/core/automation.h"
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

namespace esphome::canopen_entities {

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

// A remote state slot (OD 0x4000 + n, sub 1): a state of another node's entity, written by an RPDO from that node's
// TPDO (docs/linking-nodes.md). Read by `platform: canopen_entities` entities and toggle actions.
class RemoteSlot : public canopen::OdObject {
 public:
  uint32_t value{0};  // little endian, size bytes used
  uint8_t size{1};
  uint8_t highest{1};
  LazyCallbackManager<void(uint32_t)> listeners;

  canopen::OdResult od_write(canopen::OdStream stream, const void *buf, size_t count, size_t *done) {
    auto ret = stream.write_original(buf, count, done);
    if (ret == canopen::OdResult::OD_RESULT_OK && stream.sub() == 1)
      this->listeners.call(this->value);
    return ret;
  }
};

class CANopenEntities : public Component {
 public:
  explicit CANopenEntities(canopen::CANopenComponent *hub) : hub_(hub) {}

  void set_layout_hash(uint32_t hash) { this->layout_hash_ = hash; }
  // our node id (the hub's, from codegen)
  void set_node_id(uint8_t node_id) { this->node_id_ = node_id; }
  void enable_od_writer() { this->od_writer_ = true; }

  // OD writer (node-to-node commands, OBJECT_DICTIONARY.md "OD writer"): writes value (size 1..4 bytes, little endian)
  // to index:sub of node_id's OD in one frame on 0x500 + node_id, without a reply; our own node is written directly.
  // False if it wasn't sent / written.
  bool send_od_write(uint8_t node_id, uint16_t index, uint8_t sub, uint32_t value, uint8_t size);
  // a received OD writer frame (data / len of a 0x500..0x57F frame); public for tests
  void handle_od_writer_frame(const uint8_t *data, uint8_t len);

#ifdef CANOPEN_ENTITIES_REMOTE_SLOTS
  // remote state slots (CANOPEN_ENTITIES_REMOTE_SLOTS, all exist from the start, so listeners can be added in
  // any order): the size of slot n, from codegen; the RPDOs filling them are the hub's
  void add_remote_state(uint8_t n, uint8_t size) {
    if (n < this->slots_.size())
      this->slots_[n].size = size;
  }
  uint32_t get_remote_state(uint8_t n) const { return n < this->slots_.size() ? this->slots_[n].value : 0; }
  // callback(raw value) on every write of the slot (each RPDO frame)
  template<typename F> void add_remote_state_listener(uint8_t n, F &&callback) {
    if (n < this->slots_.size())
      this->slots_[n].listeners.add(std::forward<F>(callback));
  }
#endif
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
  // byte 0 of the frame is the target, so every node listens on all of 0x500..0x57F (other nodes' TPDO 7 COB-IDs
  // too: TPDO 7 is reserved while the OD writer is on)
  static constexpr uint32_t OD_WRITER_COB_ID = 0x500;
  static void on_od_writer_frame(void *context, uint32_t can_id, const uint8_t *data, uint8_t len) {
    static_cast<CANopenEntities *>(context)->handle_od_writer_frame(data, len);
  }
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
  uint8_t node_id_{0};
  bool od_writer_{false};
  // CANOPEN_ENTITIES_ENTITIES: from codegen, the number of entities
  StaticVector<MappedEntity, CANOPEN_ENTITIES_ENTITIES> entities_;
#ifdef CANOPEN_ENTITIES_REMOTE_SLOTS
  std::array<RemoteSlot, CANOPEN_ENTITIES_REMOTE_SLOTS> slots_{};
#endif
  // 0x2001: entity types, sub N = type of entity N
  uint8_t types_highest_{0};
  // 0x3002: TPDO layout hash (layout.py)
  uint8_t layout_highest_{1};
  uint32_t layout_hash_{0};
};

// how RemoteCommandAction encodes a command value (ENC_* in __init__.py)
enum RemoteCmdEncoding : uint8_t {
  REMOTE_CMD_RAW = 0,
  REMOTE_CMD_PERCENTAGE = 1,
  REMOTE_CMD_COLOR_TEMP = 2,
  REMOTE_CMD_SCALED = 3,
  REMOTE_CMD_REAL32 = 4,
};

uint32_t encode_remote_cmd(uint8_t encoding, float value, uint8_t size, float min_value, float max_value);

// canopen_entities.<domain>.<action>: writes one or more commands of a remote node's entity with the OD writer
template<typename... Ts> class RemoteCommandAction : public Action<Ts...>, public Parented<CANopenEntities> {
 public:
  static constexpr size_t MAX_WRITES = 3;  // MAX_WRITES in __init__.py

  void set_target(uint8_t node_id, uint8_t entity_index) {
    this->node_id_ = node_id;
    this->entity_index_ = entity_index;
  }
  void add_write(uint8_t cmd, uint8_t size, uint8_t encoding, float min_value, float max_value,
                 TemplatableValue<float, Ts...> value) {
    if (this->count_ < MAX_WRITES)
      this->writes_[this->count_++] = {cmd, size, encoding, min_value, max_value, value};
  }

  void play(const Ts &...x) override {
    for (uint8_t i = 0; i < this->count_; i++) {
      const Write &w = this->writes_[i];
      uint32_t raw = encode_remote_cmd(w.encoding, w.value.value(x...), w.size, w.min_value, w.max_value);
      this->parent_->send_od_write(this->node_id_, 0x2002 + 0x10 * this->entity_index_, w.cmd + 1, raw, w.size);
    }
  }

 protected:
  struct Write {
    uint8_t cmd;
    uint8_t size;
    uint8_t encoding;
    float min_value;
    float max_value;
    TemplatableValue<float, Ts...> value;
  };
  std::array<Write, MAX_WRITES> writes_{};
  uint8_t count_{0};
  uint8_t node_id_{0};
  uint8_t entity_index_{0};
};

}  // namespace esphome::canopen_entities

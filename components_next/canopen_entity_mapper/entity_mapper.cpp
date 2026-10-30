#include "entity_mapper.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "esphome/core/log.h"
#include "esphome/core/progmem.h"

namespace esphome::canopen_entity_mapper {

ESPHOME_LOG_TAG(TAG, "canopen_entity_mapper");

using canopen::OD_READ;
using canopen::OD_RPDO;
using canopen::OD_TPDO;
using canopen::OD_WRITE;
using canopen::OdResult;
using RecordSub = canopen::ObjectDictionary::RecordSub;

// metadata subindices, see OBJECT_DICTIONARY.md
static constexpr uint8_t META_NAME = 1;
static constexpr uint8_t META_DEVICE_CLASS = 2;
static constexpr uint8_t META_UNIT = 3;
static constexpr uint8_t META_STATE_CLASS = 4;
static constexpr uint8_t META_MIN_VALUE = 7;
static constexpr uint8_t META_MAX_VALUE = 8;

uint32_t scale_to_wire(float value, float min_val, float max_val, uint32_t max_int) {
  if (std::isnan(value))
    return max_int;
  float result = std::round((max_int - 1) * (value - min_val) / (max_val - min_val));
  return result < 0 ? 0 : (result > max_int - 1 ? max_int - 1 : (uint32_t) result);
}

float scale_from_wire(uint32_t wire, float min_val, float max_val, uint32_t max_int) {
  if (wire >= max_int)
    return NAN;
  return wire * (max_val - min_val) / (max_int - 1) + min_val;
}

void MappedEntity::init(canopen::CANopenComponent *hub, EntityBase *entity, uint8_t index, uint32_t type) {
  this->hub_ = hub;
  this->entity_ = entity;
  this->index_ = index;
  this->type_ = type;
}

void MappedEntity::add_state(uint8_t size, uint32_t value) {
  if (this->states_count_ == MAX_STATES)
    return;
  this->state_sizes_[this->states_count_] = size;
  this->states_[this->states_count_++] = value;
}

void MappedEntity::set_min_max(float min_value, float max_value) {
  this->has_min_max_ = true;
  this->min_value_ = min_value;
  this->max_value_ = max_value;
  this->meta_highest_ = META_MAX_VALUE;
}

uint32_t MappedEntity::value_to_wire(float value) const {
  switch (this->state_sizes_[0]) {
    case 1:
      return scale_to_wire(value, this->min_value_, this->max_value_, 255);
    case 2:
      return scale_to_wire(value, this->min_value_, this->max_value_, 65535);
    default: {
      uint32_t bits;
      std::memcpy(&bits, &value, sizeof(bits));
      return bits;
    }
  }
}

float MappedEntity::value_from_wire(uint32_t wire) const {
  switch (this->state_sizes_[0]) {
    case 1:
      return scale_from_wire(wire, this->min_value_, this->max_value_, 255);
    case 2:
      return scale_from_wire(wire, this->min_value_, this->max_value_, 65535);
    default: {
      float value;
      std::memcpy(&value, &wire, sizeof(value));
      return value;
    }
  }
}

size_t MappedEntity::meta_string_(uint8_t sub, std::span<char, ENTITY_NAME_BUF_SIZE> buf) const {
  buf[0] = '\0';
  switch (sub) {
    case META_NAME:
      return this->entity_->write_name_to(buf.data(), buf.size());
    case META_DEVICE_CLASS:
      return this->entity_->get_device_class_progmem().write_to(buf.data(), buf.size());
    case META_UNIT: {
      if (!this->has_unit_)
        return 0;
      StringRef unit = this->entity_->get_unit_of_measurement_ref();
      size_t len = std::min(unit.size(), buf.size() - 1);
      std::memcpy(buf.data(), unit.c_str(), len);
      buf[len] = '\0';
      return len;
    }
#ifdef USE_SENSOR
    case META_STATE_CLASS: {
      if (this->state_class_ == sensor::STATE_CLASS_NONE)
        return 0;
      auto state_class = static_cast<sensor::StateClass>(this->state_class_);
      ESPHOME_strncpy_P(buf.data(), LOG_STR_ARG(sensor::state_class_to_string(state_class)), buf.size() - 1);
      buf[buf.size() - 1] = '\0';
      return std::strlen(buf.data());
    }
#endif
    default:
      return 0;
  }
}

OdResult MappedEntity::od_read(canopen::OdStream stream, void *buf, size_t count, size_t *done) {
  uint8_t sub = stream.sub();
  if (stream.index() != this->meta_index() || sub < META_NAME || sub > META_STATE_CLASS)
    return stream.read_original(buf, count, done);
  // a metadata string: copied out of the entity (flash on ESP8266) for every chunk
  char str[ENTITY_NAME_BUF_SIZE];
  size_t len = this->meta_string_(sub, str);
  if (stream.offset() == 0)
    stream.set_size(len);  // size indication for the upload
  size_t total = std::min(stream.size(), len);
  size_t offset = stream.offset();
  size_t n = offset < total ? std::min(count, total - offset) : 0;
  std::memcpy(buf, str + offset, n);
  stream.set_offset(offset + n);
  *done = n;
  return offset + n < total ? OdResult::OD_RESULT_PARTIAL : OdResult::OD_RESULT_OK;
}

OdResult MappedEntity::od_write(canopen::OdStream stream, const void *buf, size_t count, size_t *done) {
  OdResult ret = stream.write_original(buf, count, done);
  uint8_t sub = stream.sub();
  // a whole command (expedited SDO, RPDO or write_od()) runs right away, from the hub's processing
  if (ret == OdResult::OD_RESULT_OK && stream.index() == this->commands_index() && sub >= 1 &&
      sub <= this->states_count_)
    this->command_(sub, this->commands_[sub - 1]);
  return ret;
}

void MappedEntity::command_(uint8_t sub, uint32_t value) {
  switch (this->type_ & 0xFF) {
#ifdef USE_SENSOR
    case ENTITY_TYPE_SENSOR:
    case ENTITY_TYPE_SENSOR_UINT8:
    case ENTITY_TYPE_SENSOR_UINT16:
      static_cast<sensor::Sensor *>(this->entity_)->publish_state(this->value_from_wire(value));
      break;
#endif
#ifdef USE_NUMBER
    case ENTITY_TYPE_NUMBER:
    case ENTITY_TYPE_NUMBER_UINT8:
    case ENTITY_TYPE_NUMBER_UINT16:
      static_cast<number::Number *>(this->entity_)->publish_state(this->value_from_wire(value));
      break;
#endif
#ifdef USE_BINARY_SENSOR
    case ENTITY_TYPE_BINARY_SENSOR:
      static_cast<binary_sensor::BinarySensor *>(this->entity_)->publish_state(value != 0);
      break;
#endif
#ifdef USE_SWITCH
    case ENTITY_TYPE_SWITCH:
      if (value != 0) {
        static_cast<switch_::Switch *>(this->entity_)->turn_on();
      } else {
        static_cast<switch_::Switch *>(this->entity_)->turn_off();
      }
      break;
#endif
#ifdef USE_LIGHT
    case ENTITY_TYPE_LIGHT:
      this->light_command_(sub, value);
      break;
#endif
#ifdef USE_COVER
    case ENTITY_TYPE_COVER:
      this->cover_command_(sub, value);
      break;
#endif
#ifdef USE_ALARM_CONTROL_PANEL
    case ENTITY_TYPE_ALARM:
      this->alarm_command_(value);
      break;
#endif
    default:
      break;
  }
}

void MappedEntity::add_to_od(canopen::ObjectDictionary &od) {
  // metadata: strings only if not empty (can2mqtt skips missing subindices), read by od_read()
  std::array<RecordSub, 7> meta;
  size_t n = 0;
  meta[n++] = {0, OD_READ, &this->meta_highest_, 1};
  char str[ENTITY_NAME_BUF_SIZE];
  for (uint8_t sub = META_NAME; sub <= META_STATE_CLASS; sub++) {
    if (this->meta_string_(sub, str) > 0)
      meta[n++] = {sub, OD_READ, nullptr, 0};
  }
  if (this->has_min_max_) {
    meta[n++] = {META_MIN_VALUE, OD_READ, &this->min_value_, 4};
    meta[n++] = {META_MAX_VALUE, OD_READ, &this->max_value_, 4};
  }
  od.add_record(this->meta_index(), meta.data(), n);
  od.set_object(this->meta_index(), this);

  std::array<RecordSub, MAX_STATES + 1> states;
  states[0] = {0, OD_READ, &this->states_count_, 1};
  for (uint8_t i = 0; i < this->states_count_; i++)
    states[i + 1] = {uint8_t(i + 1), OD_READ | OD_TPDO, &this->states_[i], this->state_sizes_[i]};
  od.add_record(this->states_index(), states.data(), this->states_count_ + 1);
  od.set_object(this->states_index(), this);

  // commands: one per state, same size
  std::array<RecordSub, MAX_STATES + 1> commands;
  commands[0] = {0, OD_READ, &this->states_count_, 1};
  for (uint8_t i = 0; i < this->states_count_; i++)
    commands[i + 1] = {uint8_t(i + 1), OD_WRITE | OD_RPDO, &this->commands_[i], this->state_sizes_[i]};
  od.add_record(this->commands_index(), commands.data(), this->states_count_ + 1);
  od.set_object(this->commands_index(), this);
}

void MappedEntity::set_state(uint8_t sub, uint32_t value) {
  if (sub == 0 || sub > this->states_count_ || this->states_[sub - 1] == value)
    return;
  this->states_[sub - 1] = value;
  // before the hub's setup (no OD yet) the value is just stored: the initial TPDO sends it
  this->hub_->request_tpdo(this->states_index(), sub);
}

MappedEntity *EntityMapper::add_entity_(EntityBase *entity, uint8_t index, uint32_t type) {
  if (this->entities_.size() == this->entities_.capacity()) {
    ESP_LOGE(TAG, "Entity %u: no room", index);
    return nullptr;
  }
  MappedEntity &mapped = this->entities_.emplace_next();
  mapped.init(this->hub_, entity, index, type);
  if (index > this->types_highest_)
    this->types_highest_ = index;
  return &mapped;
}

#ifdef USE_SENSOR
void EntityMapper::add_sensor(sensor::Sensor *sensor, uint8_t index, uint8_t size, float min_value,
                              float max_value) {
  uint32_t type = size == 1   ? ENTITY_TYPE_SENSOR_UINT8
                  : size == 2 ? ENTITY_TYPE_SENSOR_UINT16
                              : ENTITY_TYPE_SENSOR;
  MappedEntity *entity = this->add_entity_(sensor, index, type);
  if (entity == nullptr)
    return;
  entity->set_sensor_metadata(true, sensor->get_state_class());
  if (size != 4)
    entity->set_min_max(min_value, max_value);
  entity->add_state(size, 0);
  entity->set_state(1, entity->value_to_wire(NAN));
  sensor->add_on_state_callback([entity](float value) { entity->set_value(value); });
}
#endif

#ifdef USE_BINARY_SENSOR
void EntityMapper::add_binary_sensor(binary_sensor::BinarySensor *sensor, uint8_t index) {
  MappedEntity *entity = this->add_entity_(sensor, index, ENTITY_TYPE_BINARY_SENSOR);
  if (entity == nullptr)
    return;
  entity->add_state(1, 0);
  sensor->add_on_state_callback([entity](bool state) { entity->set_state(1, state ? 1 : 0); });
}
#endif

#ifdef USE_NUMBER
void EntityMapper::add_number(number::Number *number, uint8_t index, uint8_t size, float min_value,
                              float max_value) {
  uint32_t type = size == 1   ? ENTITY_TYPE_NUMBER_UINT8
                  : size == 2 ? ENTITY_TYPE_NUMBER_UINT16
                              : ENTITY_TYPE_NUMBER;
  MappedEntity *entity = this->add_entity_(number, index, type);
  if (entity == nullptr)
    return;
  if (size != 4)
    entity->set_min_max(min_value, max_value);
  entity->add_state(size, 0);
  entity->set_state(1, entity->value_to_wire(number->state));
  number->add_on_state_callback([entity](float value) { entity->set_value(value); });
}
#endif

#ifdef USE_SWITCH
void EntityMapper::add_switch(switch_::Switch *sw, uint8_t index) {
  MappedEntity *entity = this->add_entity_(sw, index, ENTITY_TYPE_SWITCH);
  if (entity == nullptr)
    return;
  entity->add_state(1, sw->get_initial_state_with_restore_mode().value_or(false) ? 1 : 0);
  sw->add_on_state_callback([entity](bool state) { entity->set_state(1, state ? 1 : 0); });
}
#endif

#ifdef USE_LIGHT
void EntityMapper::add_light(light::LightState *light, uint8_t index, uint32_t caps) {
  // version 1: caps in bits 16..23
  MappedEntity *entity = this->add_entity_(light, index, ENTITY_TYPE_LIGHT | (1 << 8) | (caps << 16));
  if (entity == nullptr)
    return;
  // states in layout.py order: state, brightness (brightness or color temperature), color temperature
  entity->add_state(1, 0);
  if (caps & (LIGHT_CAP_BRIGHTNESS | LIGHT_CAP_COLOR_TEMPERATURE))
    entity->add_state(1, 0);
  if (caps & LIGHT_CAP_COLOR_TEMPERATURE) {
    entity->add_state(1, 0);
    auto traits = light->get_traits();
    entity->set_min_max(traits.get_min_mireds(), traits.get_max_mireds());
  }
  entity->update_light();
  // lives as long as the light: allocated once, while the configuration is set up
  light->add_remote_values_listener(new LightListener(entity));  // NOLINT(cppcoreguidelines-owning-memory)
}

void MappedEntity::update_light() {
  const auto &values = static_cast<light::LightState *>(this->entity_)->remote_values;
  uint32_t caps = this->get_caps();
  this->set_state(1, values.get_state() > 0.0f ? 1 : 0);
  if (caps & (LIGHT_CAP_BRIGHTNESS | LIGHT_CAP_COLOR_TEMPERATURE))
    this->set_state(2, percentage_to_wire(values.get_brightness()));
  if (caps & LIGHT_CAP_COLOR_TEMPERATURE)
    this->set_state(3, color_temp_to_wire(values.get_color_temperature()));
}

// sub 1: 0 off, other on; 2: brightness (percentage); 3: color temperature (mireds scaled to 100..1000)
void MappedEntity::light_command_(uint8_t sub, uint32_t value) {
  auto call = static_cast<light::LightState *>(this->entity_)->make_call();
  if (sub == 1) {
    call.set_state(value != 0);
  } else {
    float scaled = sub == 2 ? percentage_from_wire(value) : color_temp_from_wire(value);
    if (std::isnan(scaled))
      return;
    if (sub == 2) {
      call.set_brightness_if_supported(scaled);
    } else {
      call.set_color_temperature_if_supported(scaled);
    }
  }
  call.perform();
}
#endif

#ifdef USE_COVER
// 0 open, 1 opening, 2 closed, 3 closing
static uint32_t cover_state(cover::Cover *cover) {
  switch (cover->current_operation) {
    case cover::COVER_OPERATION_OPENING:
      return 1;
    case cover::COVER_OPERATION_CLOSING:
      return 3;
    default:
      return cover->position == cover::COVER_CLOSED ? 2 : 0;
  }
}

void EntityMapper::add_cover(cover::Cover *cover, uint8_t index, uint32_t caps) {
  MappedEntity *entity = this->add_entity_(cover, index, ENTITY_TYPE_COVER | (1 << 8) | (caps << 16));
  if (entity == nullptr)
    return;
  // states in layout.py order: state, position, tilt
  entity->add_state(1, 0);
  if (caps & COVER_CAP_POSITION)
    entity->add_state(1, 0);
  if (caps & COVER_CAP_TILT)
    entity->add_state(1, 0);
  entity->update_cover();
  cover->add_on_state_callback([entity]() { entity->update_cover(); });
}

void MappedEntity::update_cover() {
  auto *cover = static_cast<cover::Cover *>(this->entity_);
  uint32_t caps = this->get_caps();
  this->set_state(1, cover_state(cover));
  uint8_t sub = 2;
  if (caps & COVER_CAP_POSITION)
    this->set_state(sub++, percentage_to_wire(cover->position));
  if (caps & COVER_CAP_TILT)
    this->set_state(sub, percentage_to_wire(cover->tilt));
}

// sub 1: 0 stop, 1 open, 2 close; then position, tilt (percentages), the ones in caps
void MappedEntity::cover_command_(uint8_t sub, uint32_t value) {
  auto call = static_cast<cover::Cover *>(this->entity_)->make_call();
  if (sub == 1) {
    if (value == 0) {
      call.set_command_stop();
    } else if (value == 1) {
      call.set_command_open();
    } else if (value == 2) {
      call.set_command_close();
    } else {
      return;
    }
  } else {
    float scaled = percentage_from_wire(value);
    if (std::isnan(scaled))
      return;
    bool position = sub == 2 && (this->get_caps() & COVER_CAP_POSITION);
    if (position) {
      call.set_position(scaled);
    } else {
      call.set_tilt(scaled);
    }
  }
  call.perform();
}
#endif

#ifdef USE_ALARM_CONTROL_PANEL
void EntityMapper::add_alarm(alarm_control_panel::AlarmControlPanel *alarm, uint8_t index) {
  MappedEntity *entity = this->add_entity_(alarm, index, ENTITY_TYPE_ALARM);
  if (entity == nullptr)
    return;
  entity->add_state(1, alarm->get_state());
  alarm->add_on_state_callback([entity](alarm_control_panel::AlarmControlPanelState state) {
    entity->set_state(1, static_cast<uint32_t>(state));
  });
}

void MappedEntity::alarm_command_(uint32_t value) {
  auto *alarm = static_cast<alarm_control_panel::AlarmControlPanel *>(this->entity_);
  switch (value) {
    case 0:
      alarm->disarm();
      break;
    case 1:
      alarm->arm_away();
      break;
    case 2:
      alarm->arm_home();
      break;
    case 3:
      alarm->arm_night();
      break;
    case 4:
      alarm->arm_vacation();
      break;
    case 5:
      alarm->arm_custom_bypass();
      break;
    case 127:
      alarm->publish_state(alarm_control_panel::ACP_STATE_TRIGGERED);
      break;
    default:
      break;
  }
}
#endif

bool EntityMapper::send_od_write(uint8_t node_id, uint16_t index, uint8_t sub, uint32_t value, uint8_t size) {
  if (size == 0 || size > 4 || node_id == 0 || node_id > 127)
    return false;
  uint8_t data[8] = {node_id, sub, uint8_t(index), uint8_t(index >> 8),
                     uint8_t(value), uint8_t(value >> 8), uint8_t(value >> 16), uint8_t(value >> 24)};
  if (node_id == this->node_id_)
    return this->hub_->write_od(index, sub, data + 4, size) == OdResult::OD_RESULT_OK;
  return this->hub_->send_frame(OD_WRITER_COB_ID + node_id, data, 4 + size);
}

void EntityMapper::handle_od_writer_frame(const uint8_t *data, uint8_t len) {
  if (len < 5 || data[0] != this->node_id_)
    return;
  uint8_t sub = data[1];
  uint16_t index = data[2] | (data[3] << 8);
  OdResult ret = this->hub_->write_od(index, sub, data + 4, len - 4);
  if (ret != OdResult::OD_RESULT_OK) {
    ESP_LOGW(TAG, "OD writer: 0x%04x:%u (%u bytes) not written: %d", index, sub, len - 4, static_cast<int>(ret));
  } else {
    ESP_LOGD(TAG, "OD writer: 0x%04x:%u written", index, sub);
  }
}

void EntityMapper::setup() {
  if (this->od_writer_)
    this->hub_->add_frame_listener(OD_WRITER_COB_ID, 0x780, on_od_writer_frame, this);
  auto &od = this->hub_->get_od();
  // 0x2001: one allocation, freed when the OD has copied it
  std::vector<RecordSub> types;
  types.reserve(this->entities_.size() + 1);
  types.push_back({0, OD_READ, &this->types_highest_, 1});
  for (auto &entity : this->entities_) {
    types.push_back({entity.get_index(), OD_READ, entity.type_storage(), 4});
    entity.add_to_od(od);
  }
  od.add_record(0x2001, types.data(), types.size());
  od.add_record(0x3002, {{0, OD_READ, &this->layout_highest_, 1}, {1, OD_READ, &this->layout_hash_, 4}});
}

void EntityMapper::dump_config() {
  ESP_LOGCONFIG(TAG, "ESPHome entity mapper:");
  ESP_LOGCONFIG(TAG, "  TPDO layout hash: %08" PRIx32, this->layout_hash_);
  ESP_LOGCONFIG(TAG, "  OD writer: %s", YESNO(this->od_writer_));
  for (auto &entity : this->entities_) {
    ESP_LOGCONFIG(TAG, "  Entity %u: type %" PRIu32, entity.get_index(), entity.get_type());
  }
}

}  // namespace esphome::canopen_entity_mapper

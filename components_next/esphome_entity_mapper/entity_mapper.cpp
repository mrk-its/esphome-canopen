#include "entity_mapper.h"

#include <cmath>
#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace esphome_entity_mapper {

static const char *const TAG = "esphome_entity_mapper";

using canopen::OD_READ;
using canopen::OD_TPDO;

// metadata subindices, see OBJECT_DICTIONARY.md
static const uint8_t META_NAME = 1;
static const uint8_t META_DEVICE_CLASS = 2;
static const uint8_t META_UNIT = 3;
static const uint8_t META_STATE_CLASS = 4;
static const uint8_t META_MIN_VALUE = 7;
static const uint8_t META_MAX_VALUE = 8;

uint32_t scale_to_wire(float value, float min_val, float max_val, uint32_t max_int) {
  if (std::isnan(value))
    return max_int;
  float result = std::round((max_int - 1) * (value - min_val) / (max_val - min_val));
  return result < 0 ? 0 : (result > max_int - 1 ? max_int - 1 : (uint32_t) result);
}

static uint32_t float_bits(float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

void MappedEntity::set_min_max(float min_value, float max_value) {
  this->has_min_max_ = true;
  this->min_value_ = float_bits(min_value);
  this->max_value_ = float_bits(max_value);
  this->meta_highest_ = META_MAX_VALUE;
}

void MappedEntity::add_to_od(canopen::ObjectDictionary &od) {
  // metadata: strings only if not empty (can2mqtt skips missing subindices)
  std::vector<canopen::ObjectDictionary::RecordSub> meta{{0, OD_READ, &this->meta_highest_, 1}};
  auto add_string = [&meta](uint8_t sub, std::string &str) {
    if (!str.empty())
      meta.push_back({sub, OD_READ, str.data(), (OD_size_t) str.size()});
  };
  add_string(META_NAME, this->name_);
  add_string(META_DEVICE_CLASS, this->device_class_);
  add_string(META_UNIT, this->unit_);
  add_string(META_STATE_CLASS, this->state_class_);
  if (this->has_min_max_) {
    meta.push_back({META_MIN_VALUE, OD_READ, &this->min_value_, 4});
    meta.push_back({META_MAX_VALUE, OD_READ, &this->max_value_, 4});
  }
  od.add_record(this->meta_index(), meta);

  this->states_count_ = this->states_.size();
  std::vector<canopen::ObjectDictionary::RecordSub> states{{0, OD_READ, &this->states_count_, 1}};
  for (size_t i = 0; i < this->states_.size(); i++)
    states.push_back({uint8_t(i + 1), OD_READ | OD_TPDO, &this->states_[i], this->state_sizes_[i]});
  od.add_record(this->states_index(), states);
  od.set_object(this->states_index(), this);
}

void MappedEntity::set_state(canopen::CANopenComponent *hub, uint8_t sub, uint32_t value) {
  if (sub == 0 || sub > this->states_.size() || this->states_[sub - 1] == value)
    return;
  this->states_[sub - 1] = value;
  // before the hub's setup (no OD yet) the value is just stored: the initial TPDO sends it
  hub->request_tpdo(this->states_index(), sub);
}

MappedEntity *EntityMapper::add_entity_(uint8_t index, uint32_t type) {
  this->entities_.push_back(make_unique<MappedEntity>(index, type));
  if (index > this->types_highest_)
    this->types_highest_ = index;
  return this->entities_.back().get();
}

#ifdef USE_SENSOR
void EntityMapper::add_sensor(sensor::Sensor *sensor, uint8_t index, uint8_t size, float min_value,
                              float max_value) {
  uint32_t type = size == 1   ? ENTITY_TYPE_SENSOR_UINT8
                  : size == 2 ? ENTITY_TYPE_SENSOR_UINT16
                              : ENTITY_TYPE_SENSOR;
  MappedEntity *entity = this->add_entity_(index, type);
  char device_class[MAX_DEVICE_CLASS_LENGTH];
  entity->set_metadata(sensor->get_name().str(), sensor->get_device_class_to(device_class),
                       sensor->get_unit_of_measurement_ref().str(),
                       LOG_STR_ARG(sensor::state_class_to_string(sensor->get_state_class())));
  auto to_wire = [size, min_value, max_value](float value) -> uint32_t {
    if (size == 1)
      return scale_to_wire(value, min_value, max_value, 255);
    if (size == 2)
      return scale_to_wire(value, min_value, max_value, 65535);
    return float_bits(value);
  };
  if (size != 4)
    entity->set_min_max(min_value, max_value);
  entity->add_state(size, to_wire(NAN));
  sensor->add_on_state_callback(
      [this, entity, to_wire](float value) { entity->set_state(this->hub_, 1, to_wire(value)); });
}
#endif

#ifdef USE_BINARY_SENSOR
void EntityMapper::add_binary_sensor(binary_sensor::BinarySensor *sensor, uint8_t index) {
  MappedEntity *entity = this->add_entity_(index, ENTITY_TYPE_BINARY_SENSOR);
  char device_class[MAX_DEVICE_CLASS_LENGTH];
  entity->set_metadata(sensor->get_name().str(), sensor->get_device_class_to(device_class), "", "");
  entity->add_state(1, 0);
  sensor->add_on_state_callback([this, entity](bool state) { entity->set_state(this->hub_, 1, state ? 1 : 0); });
}
#endif

#ifdef USE_NUMBER
void EntityMapper::add_number(number::Number *number, uint8_t index, uint8_t size, float min_value,
                              float max_value) {
  uint32_t type = size == 1   ? ENTITY_TYPE_NUMBER_UINT8
                  : size == 2 ? ENTITY_TYPE_NUMBER_UINT16
                              : ENTITY_TYPE_NUMBER;
  MappedEntity *entity = this->add_entity_(index, type);
  char device_class[MAX_DEVICE_CLASS_LENGTH];
  entity->set_metadata(number->get_name().str(), number->get_device_class_to(device_class), "", "");
  auto to_wire = [size, min_value, max_value](float value) -> uint32_t {
    if (size == 1)
      return scale_to_wire(value, min_value, max_value, 255);
    if (size == 2)
      return scale_to_wire(value, min_value, max_value, 65535);
    return float_bits(value);
  };
  if (size != 4)
    entity->set_min_max(min_value, max_value);
  entity->add_state(size, to_wire(number->state));
  number->add_on_state_callback(
      [this, entity, to_wire](float value) { entity->set_state(this->hub_, 1, to_wire(value)); });
}
#endif

#ifdef USE_SWITCH
void EntityMapper::add_switch(switch_::Switch *sw, uint8_t index) {
  MappedEntity *entity = this->add_entity_(index, ENTITY_TYPE_SWITCH);
  char device_class[MAX_DEVICE_CLASS_LENGTH];
  entity->set_metadata(sw->get_name().str(), sw->get_device_class_to(device_class), "", "");
  entity->add_state(1, sw->get_initial_state_with_restore_mode().value_or(false) ? 1 : 0);
  sw->add_on_state_callback([this, entity](bool state) { entity->set_state(this->hub_, 1, state ? 1 : 0); });
}
#endif

#ifdef USE_LIGHT
void EntityMapper::add_light(light::LightState *light, uint8_t index, uint32_t caps) {
  // version 1: caps in bits 16..23
  MappedEntity *entity = this->add_entity_(index, ENTITY_TYPE_LIGHT | (1 << 8) | (caps << 16));
  entity->set_metadata(light->get_name().str(), "", "", "");
  // states in layout.py order: state, brightness (brightness or color temperature), color temperature
  const auto &values = light->remote_values;
  entity->add_state(1, values.get_state() > 0.0f ? 1 : 0);
  if (caps & (LIGHT_CAP_BRIGHTNESS | LIGHT_CAP_COLOR_TEMPERATURE))
    entity->add_state(1, percentage_to_wire(values.get_brightness()));
  if (caps & LIGHT_CAP_COLOR_TEMPERATURE) {
    entity->add_state(1, color_temp_to_wire(values.get_color_temperature()));
    auto traits = light->get_traits();
    entity->set_min_max(traits.get_min_mireds(), traits.get_max_mireds());
  }
  this->light_listeners_.push_back(make_unique<LightListener>(this, entity, light, caps));
  light->add_remote_values_listener(this->light_listeners_.back().get());
}

void EntityMapper::LightListener::on_light_remote_values_update() {
  const auto &values = this->light_->remote_values;
  auto *hub = this->mapper_->hub_;
  this->entity_->set_state(hub, 1, values.get_state() > 0.0f ? 1 : 0);
  if (this->caps_ & (LIGHT_CAP_BRIGHTNESS | LIGHT_CAP_COLOR_TEMPERATURE))
    this->entity_->set_state(hub, 2, percentage_to_wire(values.get_brightness()));
  if (this->caps_ & LIGHT_CAP_COLOR_TEMPERATURE)
    this->entity_->set_state(hub, 3, color_temp_to_wire(values.get_color_temperature()));
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
  MappedEntity *entity = this->add_entity_(index, ENTITY_TYPE_COVER | (1 << 8) | (caps << 16));
  char device_class[MAX_DEVICE_CLASS_LENGTH];
  entity->set_metadata(cover->get_name().str(), cover->get_device_class_to(device_class), "", "");
  // states in layout.py order: state, position, tilt
  entity->add_state(1, cover_state(cover));
  uint8_t position_sub = 0, tilt_sub = 0;
  if (caps & COVER_CAP_POSITION) {
    entity->add_state(1, percentage_to_wire(cover->position));
    position_sub = 2;
  }
  if (caps & COVER_CAP_TILT) {
    entity->add_state(1, percentage_to_wire(cover->tilt));
    tilt_sub = position_sub ? 3 : 2;
  }
  cover->add_on_state_callback([this, entity, cover, position_sub, tilt_sub]() {
    entity->set_state(this->hub_, 1, cover_state(cover));
    if (position_sub)
      entity->set_state(this->hub_, position_sub, percentage_to_wire(cover->position));
    if (tilt_sub)
      entity->set_state(this->hub_, tilt_sub, percentage_to_wire(cover->tilt));
  });
}
#endif

#ifdef USE_ALARM_CONTROL_PANEL
void EntityMapper::add_alarm(alarm_control_panel::AlarmControlPanel *alarm, uint8_t index) {
  MappedEntity *entity = this->add_entity_(index, ENTITY_TYPE_ALARM);
  entity->set_metadata(alarm->get_name().str(), "", "", "");
  entity->add_state(1, alarm->get_state());
  alarm->add_on_state_callback([this, entity](alarm_control_panel::AlarmControlPanelState state) {
    entity->set_state(this->hub_, 1, static_cast<uint32_t>(state));
  });
}
#endif

void EntityMapper::setup() {
  auto &od = this->hub_->get_od();
  std::vector<canopen::ObjectDictionary::RecordSub> types{{0, OD_READ, &this->types_highest_, 1}};
  for (auto &entity : this->entities_) {
    types.push_back({entity->get_index(), OD_READ, entity->type_storage(), 4});
    entity->add_to_od(od);
  }
  od.add_record(0x2001, types);
  od.add_record(0x3002, {{0, OD_READ, &this->layout_highest_, 1}, {1, OD_READ, &this->layout_hash_, 4}});
}

void EntityMapper::dump_config() {
  ESP_LOGCONFIG(TAG, "ESPHome entity mapper:");
  ESP_LOGCONFIG(TAG, "  TPDO layout hash: %08" PRIx32, this->layout_hash_);
  for (auto &entity : this->entities_) {
    ESP_LOGCONFIG(TAG, "  Entity %u: type %" PRIu32, entity->get_index(), entity->get_type());
  }
}

}  // namespace esphome_entity_mapper
}  // namespace esphome

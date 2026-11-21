#pragma once

#include "esphome/core/defines.h"

#ifdef USE_CANOPEN_ENTITIES_LIGHT_BUTTONS

#include "canopen_entities.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"

namespace esphome::canopen_entities {

// `canopen_entities: light_buttons:` (docs/linking-nodes.md): one button controlling a dimmable light of another node,
// through the OD writer, with nothing on that node beyond the light's usual commands. A press shorter than hold_time
// toggles the light (on release). A longer one ramps the brightness while the button is held, a SetBrightness command
// every step_interval: up from min_brightness if the light is off, up at the minimum, down at full brightness,
// otherwise the opposite of the previous hold. The light's state and brightness come from remote state slots.
// The remote light should have default_transition_length: 0s, or each step starts a transition.
class LightButton : public Component {
 public:
  void set_parent(CANopenEntities *parent) { this->parent_ = parent; }
  void set_binary_sensor(binary_sensor::BinarySensor *sensor) { this->sensor_ = sensor; }
  // the remote light: node id, entity index, command index (sub - 1) of its brightness, and whether it runs the old
  // component (old OD writer frame format)
  void set_target(uint8_t node_id, uint8_t entity_index, uint8_t brightness_cmd, bool old_format) {
    this->node_id_ = node_id;
    this->entity_index_ = entity_index;
    this->brightness_cmd_ = brightness_cmd;
    this->old_format_ = old_format;
  }
  // remote state slots holding the light's state and brightness
  void set_slots(uint8_t state_slot, uint8_t brightness_slot) {
    this->state_slot_ = state_slot;
    this->brightness_slot_ = brightness_slot;
  }
  // step: brightness change per step_interval (0..1)
  void set_timing(uint32_t hold_time, uint32_t step_interval, float step, float min_brightness) {
    this->hold_time_ = hold_time;
    this->step_interval_ = step_interval;
    this->step_ = step;
    this->min_brightness_ = min_brightness;
  }

  void setup() override;
  void dump_config() override;

 protected:
  void on_state_(bool pressed);
  void start_ramp_();
  void ramp_step_();
  void send_(uint8_t cmd, uint32_t raw);

  CANopenEntities *parent_{nullptr};
  binary_sensor::BinarySensor *sensor_{nullptr};
  uint8_t node_id_{0};
  uint8_t entity_index_{0};
  uint8_t brightness_cmd_{1};
  bool old_format_{false};
  uint8_t state_slot_{0};
  uint8_t brightness_slot_{0};
  uint32_t hold_time_{400};
  uint32_t step_interval_{100};
  float step_{0.033f};
  float min_brightness_{0.02f};

  bool pressed_{false};
  bool ramping_{false};
  bool up_{true};
  float level_{0.0f};
};

}  // namespace esphome::canopen_entities

#endif  // USE_CANOPEN_ENTITIES_LIGHT_BUTTONS

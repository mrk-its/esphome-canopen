#pragma once

#include "esphome/core/defines.h"

#ifdef USE_CANOPEN_BUTTONS_LIGHTS

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/canopen_entities/canopen_entities.h"
#include "esphome/core/component.h"

namespace esphome::canopen_buttons {

// `canopen_buttons: lights:` (docs/linking-nodes.md): one button controlling a dimmable light of another node,
// through the OD writer, with nothing on that node beyond the light's usual commands. A press shorter than hold_time
// toggles the light (on release). A longer one ramps the brightness while the button is held, a SetBrightness command
// every step_interval: up from min_brightness if the light is off, up at the minimum, down at full brightness,
// otherwise the opposite of the previous hold. The light's state and brightness come from remote state slots.
// The remote light should have default_transition_length: 0s, or each step starts a transition.
class LightButton : public Component, public canopen_entities::RemoteTarget {
 public:
  void set_binary_sensor(binary_sensor::BinarySensor *sensor) { this->sensor_ = sensor; }
  // command index (sub - 1) of the light's brightness
  void set_brightness_cmd(uint8_t cmd) { this->brightness_cmd_ = cmd; }
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

  binary_sensor::BinarySensor *sensor_{nullptr};
  uint8_t brightness_cmd_{1};
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

}  // namespace esphome::canopen_buttons

#endif  // USE_CANOPEN_BUTTONS_LIGHTS

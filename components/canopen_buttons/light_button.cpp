#include "light_button.h"

#ifdef USE_CANOPEN_BUTTONS_LIGHTS

#include <algorithm>
#include <cinttypes>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome::canopen_buttons {

static const char *const TAG = "canopen_buttons.light";

static constexpr uint32_t HOLD_TIMEOUT_ID = 0;
static constexpr uint32_t STEP_INTERVAL_ID = 1;
// a remote brightness at least this close to a limit counts as at the limit (the wire has 254 steps)
static constexpr float LIMIT_TOLERANCE = 0.01f;

void LightButton::setup() {
  this->sensor_->add_on_state_callback([this](bool state) { this->on_state_(state); });
}

void LightButton::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Light button:\n"
                "  Remote light: node 0x%02X entity %u\n"
                "  Hold time: %" PRIu32 " ms, step %.1f%% every %" PRIu32 " ms, min brightness %.0f%%",
                this->target_node_id_, this->target_index_, this->hold_time_, this->step_ * 100.0f,
                this->step_interval_, this->min_brightness_ * 100.0f);
}

void LightButton::on_state_(bool pressed) {
  if (pressed == this->pressed_)
    return;
  this->pressed_ = pressed;
  if (pressed) {
    this->ramping_ = false;
    this->set_timeout(HOLD_TIMEOUT_ID, this->hold_time_, [this]() { this->start_ramp_(); });
    return;
  }
  this->cancel_timeout(HOLD_TIMEOUT_ID);
  this->cancel_interval(STEP_INTERVAL_ID);
  if (!this->ramping_) {
    // a short press: toggle
    bool on = this->remote_state_(this->state_slot_) != 0;
    this->send_command_(0, on ? 0 : 1);
  }
  this->ramping_ = false;
}

void LightButton::start_ramp_() {
  bool on = this->remote_state_(this->state_slot_) != 0;
  float level = canopen_entities::percentage_from_wire(this->remote_state_(this->brightness_slot_));
  if (!on || std::isnan(level)) {
    level = this->min_brightness_;
    this->up_ = true;
  } else if (level >= 1.0f - LIMIT_TOLERANCE) {
    this->up_ = false;
  } else if (level <= this->min_brightness_ + LIMIT_TOLERANCE) {
    this->up_ = true;
  } else {
    this->up_ = !this->up_;
  }
  this->level_ = std::clamp(level, this->min_brightness_, 1.0f);
  this->ramping_ = true;
  ESP_LOGD(TAG, "Dimming node 0x%02X entity %u %s from %.0f%%", this->target_node_id_, this->target_index_,
           this->up_ ? "up" : "down", this->level_ * 100.0f);
  // brightness alone doesn't switch an ESPHome light on
  this->send_command_(this->brightness_cmd_, canopen_entities::percentage_to_wire(this->level_));
  if (!on)
    this->send_command_(0, 1);
  this->set_interval(STEP_INTERVAL_ID, this->step_interval_, [this]() { this->ramp_step_(); });
}

void LightButton::ramp_step_() {
  float level = std::clamp(this->level_ + (this->up_ ? this->step_ : -this->step_), this->min_brightness_, 1.0f);
  if (level == this->level_) {
    // at the limit: nothing more to send while the button is held
    this->cancel_interval(STEP_INTERVAL_ID);
    return;
  }
  this->level_ = level;
  this->send_command_(this->brightness_cmd_, canopen_entities::percentage_to_wire(level));
}

}  // namespace esphome::canopen_buttons

#endif  // USE_CANOPEN_BUTTONS_LIGHTS

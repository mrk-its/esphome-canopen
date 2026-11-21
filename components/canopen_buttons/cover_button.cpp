#include "cover_button.h"

#ifdef USE_CANOPEN_BUTTONS_COVERS

#include <cinttypes>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::canopen_buttons {

static const char *const TAG = "canopen_buttons.cover";

static constexpr uint32_t STOP_REPEAT_TIMEOUT_ID = 0;
// the second stop, after the first: long enough for the first to have left the TX queue
static constexpr uint32_t STOP_REPEAT_DELAY = 50;

void CoverButton::setup() {
  if (this->open_button_ != nullptr)
    this->open_button_->add_on_state_callback([this](bool state) { this->on_button_(OPEN, state); });
  if (this->close_button_ != nullptr)
    this->close_button_->add_on_state_callback([this](bool state) { this->on_button_(CLOSE, state); });
}

void CoverButton::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Cover button:\n"
                "  Remote cover: node 0x%02X entity %u\n"
                "  Open button: %s, close button: %s\n"
                "  Double-click time: %" PRIu32 " ms",
                this->target_node_id_, this->target_index_, YESNO(this->open_button_ != nullptr),
                YESNO(this->close_button_ != nullptr), this->double_click_time_);
}

void CoverButton::on_button_(Direction direction, bool pressed) {
  bool &held = direction == OPEN ? this->open_held_ : this->close_held_;
  if (pressed == held)
    return;
  held = pressed;
  uint32_t now = millis();
  uint32_t &press_start = this->press_start_[direction - 1];
  if (pressed) {
    press_start = now;
    bool double_click = this->double_click_time_ > 0 && direction == this->click_direction_ &&
                        now - this->click_release_ <= this->double_click_time_;
    this->click_direction_ = STOP;
    // a press takes over from a cover moving all the way (a double-click starts that)
    this->latched_ = double_click ? direction : STOP;
    if (double_click)
      ESP_LOGD(TAG, "Cover node 0x%02X entity %u: double-click, all the way", this->target_node_id_,
               this->target_index_);
    // the last press wins
    this->move_(direction);
    return;
  }
  bool latched = this->latched_ == direction;
  // a short click (not the second half of a double-click) may be the first half of one
  this->click_direction_ = !latched && now - press_start <= this->double_click_time_ ? direction : STOP;
  this->click_release_ = now;
  if (latched)
    return;  // keeps moving all the way
  bool other_held = direction == OPEN ? this->close_held_ : this->open_held_;
  this->move_(other_held ? (direction == OPEN ? CLOSE : OPEN) : STOP);
}

void CoverButton::move_(Direction direction) {
  // a new command replaces a pending repeated stop
  this->cancel_timeout(STOP_REPEAT_TIMEOUT_ID);
  ESP_LOGD(TAG, "Cover node 0x%02X entity %u: %s", this->target_node_id_, this->target_index_,
           direction == OPEN ? "open" : (direction == CLOSE ? "close" : "stop"));
  this->send_command_(0, direction);
  if (direction == STOP)
    this->set_timeout(STOP_REPEAT_TIMEOUT_ID, STOP_REPEAT_DELAY, [this]() { this->send_command_(0, STOP); });
}

}  // namespace esphome::canopen_buttons

#endif  // USE_CANOPEN_BUTTONS_COVERS

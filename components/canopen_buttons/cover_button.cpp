#include "cover_button.h"

#ifdef USE_CANOPEN_BUTTONS_COVERS

#include "esphome/core/log.h"

namespace esphome::canopen_buttons {

static const char *const TAG = "canopen_buttons.cover";

static constexpr uint32_t STOP_REPEAT_TIMEOUT_ID = 0;
// the second stop, after the first: long enough for the first to have left the TX queue
static constexpr uint32_t STOP_REPEAT_DELAY = 50;

void CoverButton::setup() {
  if (this->open_sensor_ != nullptr)
    this->open_sensor_->add_on_state_callback([this](bool state) { this->on_button_(OPEN, state); });
  if (this->close_sensor_ != nullptr)
    this->close_sensor_->add_on_state_callback([this](bool state) { this->on_button_(CLOSE, state); });
}

void CoverButton::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Cover button:\n"
                "  Remote cover: node 0x%02X entity %u\n"
                "  Open button: %s, close button: %s",
                this->target_node_id_, this->target_index_, YESNO(this->open_sensor_ != nullptr),
                YESNO(this->close_sensor_ != nullptr));
}

void CoverButton::on_button_(Direction direction, bool pressed) {
  bool &held = direction == OPEN ? this->open_held_ : this->close_held_;
  if (pressed == held)
    return;
  held = pressed;
  if (pressed) {
    // the last press wins
    this->move_(direction);
    return;
  }
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

#pragma once

#include "esphome/core/defines.h"

#ifdef USE_CANOPEN_BUTTONS_COVERS

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/canopen_entities/canopen_entities.h"
#include "esphome/core/component.h"

namespace esphome::canopen_buttons {

// `canopen_buttons: covers:` (docs/linking-nodes.md): `open_button` / `close_button` (either may be missing) moving a
// cover of another node while held, through the OD writer (Cover Command: 0 stop, 1 open, 2 close). A press sends its
// direction; the last press wins. A release moves the other way if that button is still held, otherwise it stops: the
// stop goes out twice (STOP_REPEAT_DELAY apart), since OD writer frames get no reply and a lost stop would run the
// cover to its end. A double-click (the same button; the first press and the gap before the second each at most
// double_click_time) moves the cover all the way: the second release doesn't stop it. Any later press takes over
// again (a tap stops it).
class CoverButton : public Component, public canopen_entities::RemoteTarget {
 public:
  // the buttons: any binary sensors
  void set_open_button(binary_sensor::BinarySensor *button) { this->open_button_ = button; }
  void set_close_button(binary_sensor::BinarySensor *button) { this->close_button_ = button; }
  // ms; 0: no double-click
  void set_double_click_time(uint32_t ms) { this->double_click_time_ = ms; }

  void setup() override;
  void dump_config() override;

 protected:
  // Cover Command values (OBJECT_DICTIONARY.md)
  enum Direction : uint8_t { STOP = 0, OPEN = 1, CLOSE = 2 };

  void on_button_(Direction direction, bool pressed);
  void move_(Direction direction);

  binary_sensor::BinarySensor *open_button_{nullptr};
  binary_sensor::BinarySensor *close_button_{nullptr};
  bool open_held_{false};
  bool close_held_{false};
  uint32_t double_click_time_{400};
  // when each button (OPEN - 1, CLOSE - 1) was pressed
  uint32_t press_start_[2]{};
  // the last short click, the first half of a possible double-click: its button (STOP: none) and release time
  Direction click_direction_{STOP};
  uint32_t click_release_{0};
  // the direction the cover moves all the way after a double-click (STOP: none)
  Direction latched_{STOP};
};

}  // namespace esphome::canopen_buttons

#endif  // USE_CANOPEN_BUTTONS_COVERS

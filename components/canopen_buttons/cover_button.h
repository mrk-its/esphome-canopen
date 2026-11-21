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
// cover to its end.
class CoverButton : public Component, public canopen_entities::RemoteTarget {
 public:
  // the buttons: any binary sensors
  void set_open_button(binary_sensor::BinarySensor *button) { this->open_button_ = button; }
  void set_close_button(binary_sensor::BinarySensor *button) { this->close_button_ = button; }

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
};

}  // namespace esphome::canopen_buttons

#endif  // USE_CANOPEN_BUTTONS_COVERS

#pragma once

#include <string>
#include "esphome/core/defines.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace canopen {

class CanopenComponent;

// kind of a remote state, KIND_CODES in layout.py
enum RemoteStateKind : uint8_t {
  REMOTE_STATE_BOOL = 0,
  REMOTE_STATE_PERCENTAGE = 1,
  REMOTE_STATE_COLOR_TEMP = 2,
  REMOTE_STATE_COVER_STATE = 3,
  REMOTE_STATE_ALARM_STATE = 4,
  REMOTE_STATE_VALUE = 5,
};

// decoding of a raw remote state slot value
struct RemoteStateFormat {
  uint8_t kind;
  uint8_t size;
  float min_value;
  float max_value;

  float decode(uint32_t raw) const;
  std::string to_string(uint32_t raw) const;
};

// `platform: canopen` entities: show a remote state slot (CanopenComponent::add_remote_state())
#ifdef USE_BINARY_SENSOR
class CanopenRemoteBinarySensor : public binary_sensor::BinarySensor {
 public:
  void attach(CanopenComponent *canopen, uint32_t slot, uint8_t kind, uint8_t size, float min_value, float max_value);
};
#endif

#ifdef USE_SENSOR
class CanopenRemoteSensor : public sensor::Sensor {
 public:
  void attach(CanopenComponent *canopen, uint32_t slot, uint8_t kind, uint8_t size, float min_value, float max_value);
};
#endif

#ifdef USE_TEXT_SENSOR
class CanopenRemoteTextSensor : public text_sensor::TextSensor {
 public:
  void attach(CanopenComponent *canopen, uint32_t slot, uint8_t kind, uint8_t size, float min_value, float max_value);
};
#endif

}  // namespace canopen
}  // namespace esphome

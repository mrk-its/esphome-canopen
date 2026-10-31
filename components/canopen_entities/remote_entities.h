#pragma once

#include "esphome/core/defines.h"

#include "canopen_entities.h"

#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome::canopen_entities {

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
  uint8_t kind{REMOTE_STATE_BOOL};
  uint8_t size{1};
  float min_value{0.0f};
  float max_value{0.0f};

  float decode(uint32_t raw) const;
  // on / off, cover / alarm state names; nullptr for other kinds and unknown codes
  const char *name(uint32_t raw) const;
};

// `platform: canopen_entities` entities: show a remote state slot (CANopenEntities::add_remote_state())
#ifdef USE_BINARY_SENSOR
class RemoteBinarySensor : public binary_sensor::BinarySensor {
 public:
  void attach(CANopenEntities *mapper, uint8_t slot, uint8_t kind, uint8_t size, float min_value, float max_value);
};
#endif

#ifdef USE_SENSOR
class RemoteSensor : public sensor::Sensor {
 public:
  void attach(CANopenEntities *mapper, uint8_t slot, uint8_t kind, uint8_t size, float min_value, float max_value);

 protected:
  RemoteStateFormat format_;
};
#endif

#ifdef USE_TEXT_SENSOR
class RemoteTextSensor : public text_sensor::TextSensor {
 public:
  void attach(CANopenEntities *mapper, uint8_t slot, uint8_t kind, uint8_t size, float min_value, float max_value);

 protected:
  void publish_raw_(uint32_t raw);
  RemoteStateFormat format_;
};
#endif

}  // namespace esphome::canopen_entities

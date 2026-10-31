#include "remote_entities.h"

#include <cinttypes>
#include <cstdio>
#include <cstring>

namespace esphome::canopen_entities {

static const char *const COVER_STATES[] = {"open", "opening", "closed", "closing"};
static const char *const ALARM_STATES[] = {
    "disarmed", "armed_home", "armed_away", "armed_night", "armed_vacation", "armed_custom_bypass",
    "pending",  "arming",     "disarming",  "triggered"};

float RemoteStateFormat::decode(uint32_t raw) const {
  switch (this->kind) {
    case REMOTE_STATE_BOOL:
      return raw ? 1.0f : 0.0f;
    case REMOTE_STATE_PERCENTAGE:
      return percentage_from_wire(raw);
    case REMOTE_STATE_COLOR_TEMP:
      return color_temp_from_wire(raw);
    case REMOTE_STATE_VALUE:
      if (this->size == 4) {
        float value;
        std::memcpy(&value, &raw, sizeof(value));
        return value;
      }
      return scale_from_wire(raw, this->min_value, this->max_value, this->size == 1 ? 255 : 65535);
    default:  // cover / alarm state codes
      return raw;
  }
}

const char *RemoteStateFormat::name(uint32_t raw) const {
  switch (this->kind) {
    case REMOTE_STATE_BOOL:
      return raw ? "on" : "off";
    case REMOTE_STATE_COVER_STATE:
      return raw < std::size(COVER_STATES) ? COVER_STATES[raw] : nullptr;
    case REMOTE_STATE_ALARM_STATE:
      return raw < std::size(ALARM_STATES) ? ALARM_STATES[raw] : nullptr;
    default:
      return nullptr;
  }
}

#ifdef USE_BINARY_SENSOR
void RemoteBinarySensor::attach(CANopenEntities *mapper, uint8_t slot, uint8_t kind, uint8_t size, float min_value,
                                float max_value) {
  mapper->add_remote_state_listener(slot, [this](uint32_t raw) { this->publish_state(raw != 0); });
}
#endif

#ifdef USE_SENSOR
void RemoteSensor::attach(CANopenEntities *mapper, uint8_t slot, uint8_t kind, uint8_t size, float min_value,
                          float max_value) {
  this->format_ = {kind, size, min_value, max_value};
  mapper->add_remote_state_listener(slot, [this](uint32_t raw) { this->publish_state(this->format_.decode(raw)); });
}
#endif

#ifdef USE_TEXT_SENSOR
void RemoteTextSensor::attach(CANopenEntities *mapper, uint8_t slot, uint8_t kind, uint8_t size, float min_value,
                              float max_value) {
  this->format_ = {kind, size, min_value, max_value};
  mapper->add_remote_state_listener(slot, [this](uint32_t raw) { this->publish_raw_(raw); });
}

void RemoteTextSensor::publish_raw_(uint32_t raw) {
  const char *name = this->format_.name(raw);
  if (name != nullptr) {
    this->publish_state(name);
    return;
  }
  char buf[12];
  std::snprintf(buf, sizeof(buf), "%" PRIu32, raw);
  this->publish_state(buf);
}
#endif

}  // namespace esphome::canopen_entities

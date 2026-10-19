#include <cstring>
#include "esphome.h"
#include "canopen.h"
#include "remote_entities.h"

namespace esphome {
namespace canopen {

static const char *const COVER_STATES[] = {"open", "opening", "closed", "closing"};
static const char *const ALARM_STATES[] = {
    "disarmed", "armed_home", "armed_away", "armed_night", "armed_vacation", "armed_custom_bypass",
    "pending",  "arming",     "disarming",  "triggered"};

float RemoteStateFormat::decode(uint32_t raw) const {
  switch (kind) {
    case REMOTE_STATE_BOOL:
      return raw ? 1.0f : 0.0f;
    case REMOTE_STATE_PERCENTAGE:
      return percentage_from_wire(raw);
    case REMOTE_STATE_COLOR_TEMP:
      return color_temp_from_wire(raw);
    case REMOTE_STATE_VALUE:
      if (size == 4) {
        float value;
        memcpy(&value, &raw, sizeof(value));
        return value;
      }
      return scale_from_wire(raw, min_value, max_value, size == 1 ? 255 : 65535);
    default:  // cover / alarm state codes
      return raw;
  }
}

std::string RemoteStateFormat::to_string(uint32_t raw) const {
  switch (kind) {
    case REMOTE_STATE_BOOL:
      return raw ? "on" : "off";
    case REMOTE_STATE_COVER_STATE:
      if (raw < sizeof(COVER_STATES) / sizeof(COVER_STATES[0]))
        return COVER_STATES[raw];
      break;
    case REMOTE_STATE_ALARM_STATE:
      if (raw < sizeof(ALARM_STATES) / sizeof(ALARM_STATES[0]))
        return ALARM_STATES[raw];
      break;
  }
  return std::to_string(raw);
}

#ifdef USE_BINARY_SENSOR
void CanopenRemoteBinarySensor::attach(CanopenComponent *canopen, uint32_t slot, uint8_t kind, uint8_t size,
                                       float min_value, float max_value) {
  canopen->add_remote_state_listener(slot, [this](uint32_t raw) { this->publish_state(raw != 0); });
}
#endif

#ifdef USE_SENSOR
void CanopenRemoteSensor::attach(CanopenComponent *canopen, uint32_t slot, uint8_t kind, uint8_t size, float min_value,
                                 float max_value) {
  RemoteStateFormat format{kind, size, min_value, max_value};
  canopen->add_remote_state_listener(slot, [this, format](uint32_t raw) { this->publish_state(format.decode(raw)); });
}
#endif

#ifdef USE_TEXT_SENSOR
void CanopenRemoteTextSensor::attach(CanopenComponent *canopen, uint32_t slot, uint8_t kind, uint8_t size,
                                     float min_value, float max_value) {
  RemoteStateFormat format{kind, size, min_value, max_value};
  canopen->add_remote_state_listener(slot,
                                     [this, format](uint32_t raw) { this->publish_state(format.to_string(raw)); });
}
#endif

}  // namespace canopen
}  // namespace esphome

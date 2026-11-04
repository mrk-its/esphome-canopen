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
// climate codes, CLIMATE_* in layout.py (nullptr: unused code)
static const char *const CLIMATE_MODES[] = {"off", "heat_cool", "cool", "heat", "fan_only", "dry", "auto"};
static const char *const CLIMATE_ACTIONS[] = {"off",    nullptr, "cooling", "heating",
                                              "idle",   "drying", "fan",    "defrosting"};
static const char *const CLIMATE_FAN_MODES[] = {"on",     "off",    "auto",  "low",     "medium",
                                                "high",   "middle", "focus", "diffuse", "quiet"};
static const char *const CLIMATE_PRESETS[] = {"none", "home", "away", "boost", "comfort", "eco", "sleep", "activity"};
static const char *const CLIMATE_SWING_MODES[] = {"off", "both", "vertical", "horizontal"};
// climate fan mode / preset: none (or custom)
static const uint32_t CLIMATE_NONE_CODE = 255;

template<size_t N> static const char *code_name(const char *const (&names)[N], uint32_t raw) {
  return raw < N ? names[raw] : nullptr;
}

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
    case REMOTE_STATE_CLIMATE_FAN_MODE:
    case REMOTE_STATE_CLIMATE_PRESET:
      return raw == CLIMATE_NONE_CODE ? NAN : raw;
    default:  // cover / alarm state, climate mode / action / swing mode codes
      return raw;
  }
}

std::string RemoteStateFormat::to_string(uint32_t raw) const {
  const char *name = nullptr;
  switch (kind) {
    case REMOTE_STATE_BOOL:
      return raw ? "on" : "off";
    case REMOTE_STATE_COVER_STATE:
      name = code_name(COVER_STATES, raw);
      break;
    case REMOTE_STATE_ALARM_STATE:
      name = code_name(ALARM_STATES, raw);
      break;
    case REMOTE_STATE_CLIMATE_MODE:
      name = code_name(CLIMATE_MODES, raw);
      break;
    case REMOTE_STATE_CLIMATE_ACTION:
      name = code_name(CLIMATE_ACTIONS, raw);
      break;
    // 255: no (or a custom) fan mode / preset, empty text (the "none" preset is code 0)
    case REMOTE_STATE_CLIMATE_FAN_MODE:
      name = raw == CLIMATE_NONE_CODE ? "" : code_name(CLIMATE_FAN_MODES, raw);
      break;
    case REMOTE_STATE_CLIMATE_PRESET:
      name = raw == CLIMATE_NONE_CODE ? "" : code_name(CLIMATE_PRESETS, raw);
      break;
    case REMOTE_STATE_CLIMATE_SWING_MODE:
      name = code_name(CLIMATE_SWING_MODES, raw);
      break;
  }
  return name != nullptr ? name : std::to_string(raw);
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

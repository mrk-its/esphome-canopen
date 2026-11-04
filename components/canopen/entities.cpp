#include <map>
#include "esphome.h"
#include "canopen.h"
#include "fw.h"

namespace esphome {

namespace canopen {

uint32_t scale_to_wire(float value, float min_val, float max_val, uint32_t max_int) {
  if (std::isnan(value))
    return max_int;
  float result = round((max_int - 1) * (value - min_val) / (max_val - min_val));
  return result < 0 ? 0 : (result > max_int - 1 ? max_int - 1 : (uint32_t) result);
}

float scale_from_wire(uint32_t value, float min_val, float max_val, uint32_t max_int) {
  if (value == max_int)
    return NAN;
  return value * (max_val - min_val) / (max_int - 1) + min_val;
}

uint32_t percentage_to_wire(float state) { return scale_to_wire(state, 0.0, 1.0, 255); }

float percentage_from_wire(uint32_t value) { return scale_from_wire(value, 0.0, 1.0, 255); }

uint32_t color_temp_to_wire(float value) { return scale_to_wire(value, 100.0, 1000.0, 255); }

float color_temp_from_wire(uint32_t value) { return scale_from_wire(value, 100.0, 1000.0, 255); }

#ifdef USE_SENSOR
void SensorEntity::setup(CanopenComponent *canopen) {
  char dc_buf[MAX_DEVICE_CLASS_LENGTH];

  canopen->od_add_metadata(entity_id,
                           size == 1   ? ENTITY_TYPE_SENSOR_UINT8
                           : size == 2 ? ENTITY_TYPE_SENSOR_UINT16
                                       : ENTITY_TYPE_SENSOR,
                           sensor->get_name(), sensor->get_device_class_to(dc_buf), sensor->get_unit_of_measurement_ref(),
                           (char *) esphome::sensor::state_class_to_string(sensor->get_state_class()));
  canopen->od_add_min_max_metadata(entity_id, min_val, max_val);
  uint32_t state_key;

  const CO_OBJ_TYPE *type, *cmd_type;
  std::function<uint32_t(float state)> to_wire;
  std::function<float(void *buf)> from_wire;

  switch (size) {
    case 1:
      to_wire = [=, this](float state) { return scale_to_wire(state, min_val, max_val, 255); };
      from_wire = [=, this](void *buf) { return scale_from_wire(*(uint8_t *) buf, min_val, max_val, 255); };
      type = CO_TUNSIGNED8;
      cmd_type = CO_TCMD8;
      break;
    case 2:
      to_wire = [=, this](float state) { return scale_to_wire(state, min_val, max_val, 65535); };
      from_wire = [=, this](void *buf) { return scale_from_wire(*(uint16_t *) buf, min_val, max_val, 65535); };
      type = CO_TUNSIGNED16;
      cmd_type = CO_TCMD16;
      break;
    case 4:
      to_wire = [=](float state) {
        uint32_t val;
        std::memcpy(&val, &state, sizeof(val));
        return val;
      };
      from_wire = [=](void *buf) { return *(float *) buf; };
      type = CO_TUNSIGNED32;
      cmd_type = CO_TCMD32;
      break;
    default:
      ESP_LOGE(TAG, "Unsupported sensor size: %d", size);
      return;
  }
  float state = NAN;
  auto casted_state = to_wire(state);
  state_key = canopen->od_add_state(entity_id, type, &casted_state, size, tpdo);

  sensor->add_on_state_callback([=, this](float value) {
    auto casted_state = to_wire(value);
    od_set_state(canopen, state_key, &casted_state, size);
  });
  canopen->od_add_cmd(
      entity_id, [=, this](void *buffer, uint32_t size) { sensor->publish_state(from_wire(buffer)); }, cmd_type);
}
#endif

#ifdef USE_NUMBER
void NumberEntity::setup(CanopenComponent *canopen) {
  float state = number->state;
  char dc_buf[MAX_DEVICE_CLASS_LENGTH];

  canopen->od_add_metadata(entity_id,
                           size == 1   ? ENTITY_TYPE_NUMBER_UINT8
                           : size == 2 ? ENTITY_TYPE_NUMBER_UINT16
                                       : ENTITY_TYPE_NUMBER,
                           number->get_name(), number->get_device_class_to(dc_buf), "", "");

  canopen->od_add_min_max_metadata(entity_id, min_val, max_val);
  uint32_t state_key;

  const CO_OBJ_TYPE *type, *cmd_type;
  std::function<uint32_t(float state)> to_wire;
  std::function<float(void *buf)> from_wire;

  switch (size) {
    case 1:
      to_wire = [=, this](float state) { return scale_to_wire(state, min_val, max_val, 255); };
      from_wire = [=, this](void *buf) { return scale_from_wire(*(uint8_t *) buf, min_val, max_val, 255); };
      type = CO_TUNSIGNED8;
      cmd_type = CO_TCMD8;
      break;
    case 2:
      to_wire = [=, this](float state) { return scale_to_wire(state, min_val, max_val, 65535); };
      from_wire = [=, this](void *buf) { return scale_from_wire(*(uint16_t *) buf, min_val, max_val, 65535); };
      type = CO_TUNSIGNED16;
      cmd_type = CO_TCMD16;
      break;
    case 4:
      to_wire = [=](float state) {
        uint32_t val;
        std::memcpy(&val, &state, sizeof(val));
        return val;
      };
      from_wire = [=](void *buf) { return *(float *) buf; };
      type = CO_TUNSIGNED32;
      cmd_type = CO_TCMD32;
      break;
    default:
      ESP_LOGE(TAG, "Unsupported number size: %d", size);
      return;
  }
  auto casted_state = to_wire(state);
  state_key = canopen->od_add_state(entity_id, type, &casted_state, size, tpdo);
  number->add_on_state_callback([=, this](float value) {
    auto casted_state = to_wire(value);
    od_set_state(canopen, state_key, &casted_state, size);
  });
  canopen->od_add_cmd(
      entity_id, [=, this](void *buffer, uint32_t size) { number->publish_state(from_wire(buffer)); }, cmd_type);
}
#endif

#ifdef USE_BINARY_SENSOR

void BinarySensorEntity::setup(CanopenComponent *canopen) {
  char dc_buf[MAX_DEVICE_CLASS_LENGTH];

  canopen->od_add_metadata(entity_id, ENTITY_TYPE_BINARY_SENSOR, sensor->get_name(),
                           sensor->get_device_class_to(dc_buf), "",
                           "");
  auto state_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &sensor->state, 1, tpdo);
  sensor->add_on_state_callback([=, this](bool x) { od_set_state(canopen, state_key, &x, 1); });
  canopen->od_add_cmd(
      entity_id, [=, this](void *buffer, uint32_t size) { sensor->publish_state(*(uint8_t *) buffer); }, CO_TCMD8);
}

#endif

#ifdef USE_SWITCH
void SwitchEntity::setup(CanopenComponent *canopen) {
  char dc_buf[MAX_DEVICE_CLASS_LENGTH];

  auto state = switch_->get_initial_state_with_restore_mode().value_or(false);
  canopen->od_add_metadata(entity_id, ENTITY_TYPE_SWITCH, switch_->get_name(),
                           switch_->get_device_class_to(dc_buf), "", "");
  auto state_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &state, 1, tpdo);
  switch_->add_on_state_callback([=, this](bool value) { od_set_state(canopen, state_key, &value, 1); });
  canopen->od_add_cmd(entity_id, [=, this](void *buffer, uint32_t size) {
    if (((uint8_t *) buffer)[0]) {
      switch_->turn_on();
    } else {
      switch_->turn_off();
    }
  });
}
#endif

#ifdef USE_LIGHT
void LightStateEntity::on_light_remote_values_update() {
  bool state = bool(light->remote_values.get_state());
  od_set_state(canopen, state_key, &state, 1);
  if (brightness_key) {
    uint8_t brightness = percentage_to_wire(light->remote_values.get_brightness());
    od_set_state(canopen, brightness_key, &brightness, 1);
  }
  if (colortemp_key) {
    uint8_t colortemp = color_temp_to_wire(light->remote_values.get_color_temperature());
    od_set_state(canopen, colortemp_key, &colortemp, 1);
  }
}

void LightStateEntity::setup(CanopenComponent *canopen) {
  this->canopen = canopen;
  bool state = bool(light->remote_values.get_state());
  uint8_t brightness = percentage_to_wire(light->remote_values.get_brightness());
  uint8_t colortemp = color_temp_to_wire(light->remote_values.get_color_temperature());

  state_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &state, 1, tpdo);
  canopen->od_add_cmd(
      entity_id, [this](void *buffer, uint32_t size) { light->make_call().set_state(*(uint8_t *) buffer).perform(); });

  uint32_t version = 1;
  auto traits = light->get_traits();

  // caps (and so the OD layout) come from codegen, traits are only checked
  uint32_t traits_caps = 0;
  if (traits.supports_color_mode(light::ColorMode::ON_OFF)) {
    traits_caps |= 1;
  }
  // color temperature implies brightness, the two bits are exclusive (same as layout.py)
  if (traits.supports_color_capability(light::ColorCapability::COLOR_TEMPERATURE) ||
      traits.supports_color_capability(light::ColorCapability::COLD_WARM_WHITE)) {
    traits_caps |= 4;
  } else if (traits.supports_color_capability(light::ColorCapability::BRIGHTNESS)) {
    traits_caps |= 2;
  }
  if (traits_caps != caps) {
    ESP_LOGW(TAG, "light '%s': caps from config 0x%02lx differ from traits 0x%02lx", light->get_name().c_str(),
             (unsigned long) caps, (unsigned long) traits_caps);
  }

  canopen->od_add_metadata(entity_id, ENTITY_TYPE_LIGHT | (version << 8) | (caps << 16), light->get_name(), "", "", "");

  if (caps & (2 | 4)) {
    brightness_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &brightness, 1, tpdo);
    canopen->od_add_cmd(entity_id, [this](void *buffer, uint32_t size) {
      light->make_call().set_brightness_if_supported(percentage_from_wire(*(uint8_t *) buffer)).perform();
    });
  }

  if (caps & 4) {
    colortemp_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &colortemp, 1, tpdo);
    canopen->od_add_cmd(entity_id, [=, this](void *buffer, uint32_t size) {
      light->make_call().set_color_temperature_if_supported(color_temp_from_wire(*(uint8_t *) buffer)).perform();
    });

    float min_mireds = traits.get_min_mireds();
    float max_mireds = traits.get_max_mireds();

    ESP_LOGD(TAG, "min_mireds: %f, max_mireds: %f", min_mireds, max_mireds);
    canopen->od_add_min_max_metadata(entity_id, min_mireds, max_mireds);
  }
  light->add_remote_values_listener(this);
}
#endif

#ifdef USE_COVER
uint8_t get_cover_state(esphome::cover::Cover *cover) {
  switch (cover->current_operation) {
    case esphome::cover::COVER_OPERATION_OPENING:
      return 1;
    case esphome::cover::COVER_OPERATION_CLOSING:
      return 3;
    case esphome::cover::COVER_OPERATION_IDLE:
      return cover->position == esphome::cover::COVER_CLOSED ? 2 : 0;
  };
  return 0;
}

void CoverEntity::setup(CanopenComponent *canopen) {
  char dc_buf[MAX_DEVICE_CLASS_LENGTH];
  uint8_t state = get_cover_state(cover);

  auto traits = cover->get_traits();

  uint32_t version = 1;
  uint32_t pos_key = 0;
  uint32_t tilt_key = 0;

  // caps (and so the OD layout) come from codegen, traits are only checked
  uint32_t traits_caps = (traits.get_supports_position() ? 1 : 0) | (traits.get_supports_tilt() ? 2 : 0);
  if (traits_caps != caps) {
    ESP_LOGW(TAG, "cover '%s': caps from config 0x%02lx differ from traits 0x%02lx", cover->get_name().c_str(),
             (unsigned long) caps, (unsigned long) traits_caps);
  }

  canopen->od_add_metadata(entity_id, ENTITY_TYPE_COVER | (version << 8) | (caps << 16), cover->get_name(),
                           cover->get_device_class_to(dc_buf), "", "");
  auto state_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &state, 1, tpdo);

  canopen->od_add_cmd(entity_id, [this](void *buffer, uint32_t size) {
    uint8_t cmd = *(uint8_t *) buffer;
    auto call = cover->make_call();
    if (cmd == 0) {
      call.set_command_stop();
      call.perform();
    } else if (cmd == 1) {
      call.set_command_open();
      call.perform();
    } else if (cmd == 2) {
      call.set_command_close();
      call.perform();
    }
  });

  if (caps & 1) {
    uint8_t position = percentage_to_wire(cover->position);
    pos_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &position, 1, tpdo);
    canopen->od_add_cmd(entity_id, [this](void *buffer, uint32_t size) {
      float position = percentage_from_wire(*(uint8_t *) buffer);
      auto call = cover->make_call();
      call.set_position(position);
      call.perform();
    });
  }

  if (caps & 2) {
    uint8_t tilt = percentage_to_wire(cover->tilt);
    tilt_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &tilt, 1, tpdo);
    canopen->od_add_cmd(entity_id, [this](void *buffer, uint32_t size) {
      float tilt = percentage_from_wire(*(uint8_t *) buffer);
      auto call = cover->make_call();
      call.set_tilt(tilt);
      call.perform();
    });
  }

  cover->add_on_state_callback([=, this]() {
    ESP_LOGD(TAG, "on_state callback, op: %s, pos: %f", cover_operation_to_str(cover->current_operation),
             cover->position);
    uint8_t state = get_cover_state(cover);
    od_set_state(canopen, state_key, &state, 1);
    if (pos_key) {
      uint8_t position = percentage_to_wire(cover->position);
      od_set_state(canopen, pos_key, &position, 1);
    }
    if (tilt_key) {
      uint8_t tilt = percentage_to_wire(cover->tilt);
      od_set_state(canopen, tilt_key, &tilt, 1);
    }
  });
}
#endif

#ifdef USE_ALARM_CONTROL_PANEL
void AlarmEntity::setup(CanopenComponent *canopen) {
  canopen->od_add_metadata(entity_id, ENTITY_TYPE_ALARM, alarm->get_name(), "", "", "");
  auto state = alarm->get_state();
  ESP_LOGI(TAG, "Alarm initial state: %d", state);

  auto state_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &state, 1, tpdo);
  alarm->add_on_state_callback([=](alarm_control_panel::AlarmControlPanelState state) {
    od_set_state(canopen, state_key, &state, 1);
  });

  canopen->od_add_cmd(entity_id, [=](void *buffer, uint32_t size) {
    auto cmd = ((uint8_t *) buffer)[0];
    switch (cmd) {
      case 0:
        alarm->disarm();
        break;
      case 1:
        alarm->arm_away();
        break;
      case 2:
        alarm->arm_home();
        break;
      case 3:
        alarm->arm_night();
        break;
      case 4:
        alarm->arm_vacation();
        break;
      case 5:
        alarm->arm_custom_bypass();
        break;
      case 127:
        alarm->publish_state(esphome::alarm_control_panel::AlarmControlPanelState::ACP_STATE_TRIGGERED);
        break;
    }
  });
}
#endif

#ifdef USE_CLIMATE
static uint8_t climate_fan_mode_to_wire(esphome::climate::Climate *climate) {
  return climate->fan_mode.has_value() ? (uint8_t) *climate->fan_mode : CLIMATE_NONE;
}

static uint8_t climate_preset_to_wire(esphome::climate::Climate *climate) {
  return climate->preset.has_value() ? (uint8_t) *climate->preset : CLIMATE_NONE;
}

uint32_t ClimateEntity::temperature_to_wire(float value) {
  if (size == 4) {
    uint32_t raw;
    std::memcpy(&raw, &value, sizeof(raw));
    return raw;
  }
  return scale_to_wire(value, min_val, max_val, size == 1 ? 255 : 65535);
}

float ClimateEntity::temperature_from_wire(void *buffer) {
  switch (size) {
    case 1:
      return scale_from_wire(*(uint8_t *) buffer, min_val, max_val, 255);
    case 2:
      return scale_from_wire(*(uint16_t *) buffer, min_val, max_val, 65535);
    default: {
      float value;
      std::memcpy(&value, buffer, sizeof(value));
      return value;
    }
  }
}

uint32_t ClimateEntity::add_temperature_state(float value) {
  uint32_t raw = temperature_to_wire(value);
  return canopen_->od_add_state(entity_id,
                                size == 1   ? CO_TUNSIGNED8
                                : size == 2 ? CO_TUNSIGNED16
                                            : CO_TUNSIGNED32,
                                &raw, size, tpdo);
}

void ClimateEntity::add_temperature_cmd(std::function<void(float)> &&set) {
  canopen_->od_add_cmd(
      entity_id,
      [this, set = std::move(set)](void *buffer, uint32_t) {
        float value = temperature_from_wire(buffer);
        if (!std::isnan(value))  // 255 / 65535: unknown
          set(value);
      },
      size == 1   ? CO_TCMD8
      : size == 2 ? CO_TCMD16
                  : CO_TCMD32);
}

void ClimateEntity::update_states() {
  uint8_t mode = climate->mode;
  od_set_state(canopen_, mode_key_, &mode, 1);

  auto set_temperature = [this](uint32_t key, float value) {
    if (key) {
      uint32_t raw = temperature_to_wire(value);
      od_set_state(canopen_, key, &raw, size);
    }
  };
  set_temperature(target_key_, climate->target_temperature);
  set_temperature(target_low_key_, climate->target_temperature_low);
  set_temperature(target_high_key_, climate->target_temperature_high);

  if (fan_mode_key_) {
    uint8_t fan_mode = climate_fan_mode_to_wire(climate);
    od_set_state(canopen_, fan_mode_key_, &fan_mode, 1);
  }
  if (preset_key_) {
    uint8_t preset = climate_preset_to_wire(climate);
    od_set_state(canopen_, preset_key_, &preset, 1);
  }
  if (swing_mode_key_) {
    uint8_t swing_mode = climate->swing_mode;
    od_set_state(canopen_, swing_mode_key_, &swing_mode, 1);
  }

  set_temperature(current_key_, climate->current_temperature);
  uint8_t action = climate->action;
  od_set_state(canopen_, action_key_, &action, 1);
}

void ClimateEntity::setup(CanopenComponent *canopen) {
  namespace cl = esphome::climate;
  this->canopen_ = canopen;
  char dc_buf[MAX_DEVICE_CLASS_LENGTH];
  uint32_t version = 1;

  // caps (and so the OD layout) come from codegen (the `caps:` option), traits are only checked
  auto traits = climate->get_traits();
  uint32_t traits_caps = 0;
  if (traits.has_feature_flags(cl::CLIMATE_SUPPORTS_TWO_POINT_TARGET_TEMPERATURE |
                               cl::CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE))
    traits_caps |= CLIMATE_CAP_TWO_POINT;
  if (traits.get_supports_fan_modes())
    traits_caps |= CLIMATE_CAP_FAN_MODE;
  if (traits.get_supports_presets())
    traits_caps |= CLIMATE_CAP_PRESET;
  if (traits.get_supports_swing_modes())
    traits_caps |= CLIMATE_CAP_SWING_MODE;
  // leaving out a supported capability is fine, exposing an unsupported one or a single target
  // temperature of a two-point only climate is not
  bool needs_two_point = traits.has_feature_flags(cl::CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE) &&
                         !(caps & CLIMATE_CAP_TWO_POINT);
  if ((caps & ~traits_caps) || needs_two_point) {
    ESP_LOGW(TAG, "climate '%s': caps from config 0x%02lx don't match traits 0x%02lx", climate->get_name().c_str(),
             (unsigned long) caps, (unsigned long) traits_caps);
  }

  canopen->od_add_metadata(entity_id, ENTITY_TYPE_CLIMATE | (version << 8) | (caps << 16), climate->get_name(),
                           climate->get_device_class_to(dc_buf), "°C", "");
  canopen->od_add_min_max_metadata(entity_id, min_val, max_val);

  // writable states first, in the order of layout.py: a state's position is also its command index
  uint8_t mode = climate->mode;
  mode_key_ = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &mode, 1, tpdo);
  canopen->od_add_cmd(entity_id, [this](void *buffer, uint32_t) {
    uint8_t value = *(uint8_t *) buffer;
    if (value <= cl::CLIMATE_MODE_AUTO)
      climate->make_call().set_mode((cl::ClimateMode) value).perform();
  });

  if (caps & CLIMATE_CAP_TWO_POINT) {
    target_low_key_ = add_temperature_state(climate->target_temperature_low);
    add_temperature_cmd([this](float value) { climate->make_call().set_target_temperature_low(value).perform(); });
    target_high_key_ = add_temperature_state(climate->target_temperature_high);
    add_temperature_cmd([this](float value) { climate->make_call().set_target_temperature_high(value).perform(); });
  } else {
    target_key_ = add_temperature_state(climate->target_temperature);
    add_temperature_cmd([this](float value) { climate->make_call().set_target_temperature(value).perform(); });
  }

  if (caps & CLIMATE_CAP_FAN_MODE) {
    uint8_t fan_mode = climate_fan_mode_to_wire(climate);
    fan_mode_key_ = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &fan_mode, 1, tpdo);
    canopen->od_add_cmd(entity_id, [this](void *buffer, uint32_t) {
      uint8_t value = *(uint8_t *) buffer;
      if (value <= cl::CLIMATE_FAN_QUIET)
        climate->make_call().set_fan_mode((cl::ClimateFanMode) value).perform();
    });
  }

  if (caps & CLIMATE_CAP_PRESET) {
    uint8_t preset = climate_preset_to_wire(climate);
    preset_key_ = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &preset, 1, tpdo);
    canopen->od_add_cmd(entity_id, [this](void *buffer, uint32_t) {
      uint8_t value = *(uint8_t *) buffer;
      if (value <= cl::CLIMATE_PRESET_ACTIVITY)
        climate->make_call().set_preset((cl::ClimatePreset) value).perform();
    });
  }

  if (caps & CLIMATE_CAP_SWING_MODE) {
    uint8_t swing_mode = climate->swing_mode;
    swing_mode_key_ = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &swing_mode, 1, tpdo);
    canopen->od_add_cmd(entity_id, [this](void *buffer, uint32_t) {
      uint8_t value = *(uint8_t *) buffer;
      if (value <= cl::CLIMATE_SWING_HORIZONTAL)
        climate->make_call().set_swing_mode((cl::ClimateSwingMode) value).perform();
    });
  }

  // read only states, no command
  current_key_ = add_temperature_state(climate->current_temperature);
  uint8_t action = climate->action;
  action_key_ = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &action, 1, tpdo);

  climate->add_on_state_callback([this](cl::Climate &) { this->update_states(); });
}
#endif

}  // namespace canopen
}  // namespace esphome

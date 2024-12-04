#include <map>
#include "esphome.h"
#include "canopen.h"
#include "fw.h"

namespace esphome {

namespace canopen {


float scale_to_wire(float value, float min_val, float max_val, float n_levels) {
  if (std::isnan(value))
    return n_levels;
  float result = n_levels * (value - min_val) / (max_val - min_val + 1);
  return result < 0 ? 0 : (result > n_levels - 1 ? n_levels - 1 : result);
}

float scale_from_wire(float value, float min_val, float max_val, float n_levels) {
  if (int(value) == int(n_levels))
    return NAN;
  return value * (max_val - min_val + 1) / n_levels + min_val;
}

#ifdef USE_SENSOR
void SensorEntity::setup(CanopenComponent *canopen) {
  canopen->od_add_metadata(entity_id,
                           size == 1   ? ENTITY_TYPE_SENSOR_UINT8
                           : size == 2 ? ENTITY_TYPE_SENSOR_UINT16
                                       : ENTITY_TYPE_SENSOR,
                           sensor->get_name(), sensor->get_device_class(), sensor->get_unit_of_measurement(),
                           esphome::sensor::state_class_to_string(sensor->get_state_class()));
  canopen->od_add_sensor_metadata(entity_id, min_val, max_val);
  uint32_t state_key;

  const CO_OBJ_TYPE *type, *cmd_type;
  std::function<uint32_t(float state)> to_wire;
  std::function<float(void *buf)> from_wire;

  switch (size) {
    case 1:
      to_wire = [=](float state) { return (uint8_t) scale_to_wire(state, min_val, max_val, 255); };
      from_wire = [=](void *buf) { return scale_from_wire((float) *(uint8_t *) buf, min_val, max_val, 255); };
      type = CO_TUNSIGNED8;
      cmd_type = CO_TCMD8;
      break;
    case 2:
      to_wire = [=](float state) { return (uint16_t) scale_to_wire(state, min_val, max_val, 65535); };
      from_wire = [=](void *buf) { return scale_from_wire((float) *(uint16_t *) buf, min_val, max_val, 65535); };
      type = CO_TUNSIGNED16;
      cmd_type = CO_TCMD16;
      break;
    case 4:
      to_wire = [=](float state) { return *(uint32_t *) &state; };
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

  sensor->add_on_state_callback([=](float value) {
    auto casted_state = to_wire(value);
    od_set_state(canopen, state_key, &casted_state, size);
  });
  canopen->od_add_cmd(
      entity_id, [=](void *buffer, uint32_t size) { sensor->publish_state(from_wire(buffer)); }, cmd_type);
}
#endif

#ifdef USE_NUMBER
void NumberEntity::setup(CanopenComponent *canopen) {
  float state = number->state;
  canopen->od_add_metadata(entity_id,
                           size == 1   ? ENTITY_TYPE_NUMBER_UINT8
                           : size == 2 ? ENTITY_TYPE_NUMBER_UINT16
                                       : ENTITY_TYPE_NUMBER,
                           number->get_name(), number->traits.get_device_class(), "", "");

  canopen->od_add_sensor_metadata(entity_id, min_val, max_val);
  uint32_t state_key;

  const CO_OBJ_TYPE *type, *cmd_type;
  std::function<uint32_t(float state)> to_wire;
  std::function<float(void *buf)> from_wire;

  switch (size) {
    case 1:
      to_wire = [=](float state) { return (uint8_t) scale_to_wire(state, min_val, max_val, 255); };
      from_wire = [=](void *buf) { return scale_from_wire((float) *(uint8_t *) buf, min_val, max_val, 255); };
      type = CO_TUNSIGNED8;
      cmd_type = CO_TCMD8;
      break;
    case 2:
      to_wire = [=](float state) { return (uint16_t) scale_to_wire(state, min_val, max_val, 65535); };
      from_wire = [=](void *buf) { return scale_from_wire((float) *(uint16_t *) buf, min_val, max_val, 65535); };
      type = CO_TUNSIGNED16;
      cmd_type = CO_TCMD16;
      break;
    case 4:
      to_wire = [=](float state) { return *(uint32_t *) &state; };
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
  number->add_on_state_callback([=](float value) {
    auto casted_state = to_wire(value);
    od_set_state(canopen, state_key, &casted_state, size);
  });
  canopen->od_add_cmd(
      entity_id, [=](void *buffer, uint32_t size) { number->publish_state(from_wire(buffer)); }, cmd_type);
}
#endif

#ifdef USE_BINARY_SENSOR

void BinarySensorEntity::setup(CanopenComponent *canopen) {
  canopen->od_add_metadata(entity_id, ENTITY_TYPE_BINARY_SENSOR, sensor->get_name(), sensor->get_device_class(), "",
                           "");
  auto state_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &sensor->state, 1, tpdo);
  sensor->add_on_state_callback([=](bool x) { od_set_state(canopen, state_key, &x, 1); });
  canopen->od_add_cmd(
      entity_id, [=](void *buffer, uint32_t size) { sensor->publish_state(*(uint8_t *)buffer); }, CO_TCMD8);

}

#endif

#ifdef USE_SWITCH
void SwitchEntity::setup(CanopenComponent *canopen) {
  auto state = switch_->get_initial_state_with_restore_mode().value_or(false);
  canopen->od_add_metadata(entity_id, ENTITY_TYPE_SWITCH, switch_->get_name(), switch_->get_device_class(), "", "");
  auto state_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &state, 1, tpdo);
  switch_->add_on_state_callback([=](bool value) { od_set_state(canopen, state_key, &value, 1); });
  canopen->od_add_cmd(entity_id, [=](void *buffer, uint32_t size) {
    if (((uint8_t *) buffer)[0]) {
      switch_->turn_on();
    } else {
      switch_->turn_off();
    }
  });
}
#endif


#ifdef USE_LIGHT
void LightStateEntity::setup(CanopenComponent *canopen) {
  bool state = bool(light->remote_values.get_state());
  uint8_t brightness = (uint8_t) (light->remote_values.get_brightness() * 255);
  uint16_t colortemp = (uint16_t) (light->remote_values.get_color_temperature());
  canopen->od_add_metadata(entity_id, ENTITY_TYPE_LIGHT, light->get_name(), "", "", "");
  auto state_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &state, 1, tpdo);
  auto brightness_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &brightness, 1, tpdo);
  auto colortemp_key = canopen->od_add_state(entity_id, CO_TUNSIGNED16, &colortemp, 2, tpdo);
  light->add_new_remote_values_callback([=]() {
    bool value = bool(light->remote_values.get_state());
    uint8_t brightness = (uint8_t) (light->remote_values.get_brightness() * 255);
    uint16_t colortemp = (uint16_t) (light->remote_values.get_color_temperature());
    od_set_state(canopen, state_key, &value, 1);
    od_set_state(canopen, brightness_key, &brightness, 1);
    od_set_state(canopen, colortemp_key, &colortemp, 2);
  });
  canopen->od_add_cmd(
      entity_id, [=](void *buffer, uint32_t size) { light->make_call().set_state(((uint8_t *) buffer)[0]).perform(); });
  canopen->od_add_cmd(entity_id, [=](void *buffer, uint32_t size) {
    light->make_call().set_brightness_if_supported(float(((uint8_t *) buffer)[0]) / 255.0).perform();
  });
  canopen->od_add_cmd(
      entity_id,
      [=](void *buffer, uint32_t size) {
        light->make_call().set_color_temperature_if_supported(float(((uint16_t *) buffer)[0])).perform();
      },
      CO_TCMD16);
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
  uint8_t state = get_cover_state(cover);
  uint8_t position = uint8_t(cover->position * 255);
  canopen->od_add_metadata(entity_id, ENTITY_TYPE_COVER, cover->get_name(), cover->get_device_class(), "", "");
  auto state_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &state, 1, tpdo);
  auto pos_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &position, 1, tpdo);
  cover->add_on_state_callback([=]() {
    ESP_LOGD(TAG, "on_state callback, op: %s, pos: %f", cover_operation_to_str(cover->current_operation),
             cover->position);
    uint8_t position = uint8_t(cover->position * 255);
    uint8_t state = get_cover_state(cover);
    od_set_state(canopen, state_key, &state, 1);
    od_set_state(canopen, pos_key, &position, 1);
  });
  canopen->od_add_cmd(entity_id, [=](void *buffer, uint32_t size) {
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
  canopen->od_add_cmd(entity_id, [=](void *buffer, uint32_t size) {
    uint8_t cmd = *(uint8_t *) buffer;
    float position = ((float) cmd) / 255.0;
    auto call = cover->make_call();
    call.set_position(position);
    call.perform();
  });
}
#endif

#ifdef USE_ALARM_CONTROL_PANEL
void AlarmEntity::setup(CanopenComponent *canopen) {
  canopen->od_add_metadata(entity_id, ENTITY_TYPE_ALARM, alarm->get_name(), "", "", "");
  auto state = alarm->get_state();
  ESP_LOGI(TAG, "Alarm initial state: %d", state);

  auto state_key = canopen->od_add_state(entity_id, CO_TUNSIGNED8, &state, 1, tpdo);
  alarm->add_on_state_callback([=]() {
    auto state = alarm->get_state();
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

}

}
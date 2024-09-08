#include <map>
#include "esphome.h"
#include "canopen.h"
#include "fw.h"

extern "C" {
void esp_log(const char *tag, const char *fmt, ...) {
  va_list arglist;
  va_start(arglist, fmt);
  esp_log_vprintf_(0, tag, __LINE__, fmt, arglist);
  va_end(arglist);
}
}

void CONmtHbConsEvent(CO_NMT *nmt, uint8_t nodeId) {
  if (esphome::global_canopen && esphome::global_canopen->on_hb_cons_event) {
    esphome::global_canopen->on_hb_cons_event->trigger(nodeId);
  };
}

namespace esphome {

canopen::CanopenComponent *global_canopen = 0;

namespace canopen {

#ifdef USE_MQTT
mqtt::MQTTClientComponent *global_mqtt_client = nullptr;

class CustomMQTTMessageTrigger : public Trigger<std::string, std::string>, public Component {
 public:
  CustomMQTTMessageTrigger(std::string topic) : topic_(std::move(topic)) {}
  void setup() override {
    global_mqtt_client->subscribe(
        this->topic_,
        [this](const std::string &topic, const std::string &payload) {
          if (this->payload_.has_value() && payload != *this->payload_) {
            return;
          }
          this->trigger(topic, payload);
        },
        this->qos_);
  }

  void set_qos(uint8_t qos) { this->qos_ = qos; }
  void set_payload(const std::string &payload) { this->payload_ = payload; }
  // void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

 protected:
  std::string topic_;
  uint8_t qos_{0};
  optional<std::string> payload_;
};
#endif

// extern "C" {
//   void log_printf(char *fmt, ...) {
//     ESP_LOGD(TAG, fmt);
//   }

// }

CO_OBJ_STR *od_string(const std::string &str) {
  auto od_str = new CO_OBJ_STR();
  od_str->Offset = 0;
  od_str->Start = (uint8_t *) (new std::string(str))->c_str();
  return od_str;
}

CanopenComponent::CanopenComponent(uint32_t node_id) {
  ESP_LOGI(TAG, "initializing CANopen-stack, node_id: %03lx", node_id);
  memset(rpdo_buf, 0, sizeof(rpdo_buf));
  this->node_id = node_id;
  NodeSpec.NodeId = node_id;
  CODictInit(&node.Dict, &node, NodeSpec.Dict, NodeSpec.DictLen);

  global_canopen = this;
  on_operational = 0;
  on_pre_operational = 0;
  on_hb_cons_event = 0;

  state_update_delay_us = 0;
  heartbeat_interval_ms = 0;
  memset(&status, 0, sizeof(status));
  memset(&last_status, 0, sizeof(last_status));
  CO_OBJ_STR *esphome_ver_str = od_string(ESPHOME_VERSION " " + App.get_compilation_time());
  ODAddUpdate(NodeSpec.Dict, CO_KEY(0x100a, 0, CO_OBJ_____R_), CO_TSTRING, (CO_DATA) esphome_ver_str);
}
void CanopenComponent::set_state_update_delay(uint32_t delay_us) { state_update_delay_us = delay_us; }
void CanopenComponent::set_heartbeat_interval(uint16_t interval_ms) { heartbeat_interval_ms = interval_ms; }

void CanopenComponent::on_frame(uint32_t can_id, bool rtr, std::vector<uint8_t> &data) {
  recv_frame = {{can_id, {}, (uint8_t) data.size()}};
  memcpy(recv_frame.value().Data, &data[0], data.size());
  CONodeProcess(&node);
}

void CanopenComponent::set_canbus(canbus::Canbus *canbus) {
  Automation<std::vector<uint8_t>, uint32_t, bool> *automation;
  LambdaAction<std::vector<uint8_t>, uint32_t, bool> *lambdaaction;
  canbus::CanbusTrigger *canbus_canbustrigger;

  this->canbus = canbus;

  canbus_canbustrigger = new canbus::CanbusTrigger(canbus, 0, 0, false);
  canbus_canbustrigger->set_component_source("canbus");
  App.register_component(canbus_canbustrigger);
  automation = new Automation<std::vector<uint8_t>, uint32_t, bool>(canbus_canbustrigger);
  auto cb = [this](std::vector<uint8_t> x, uint32_t can_id, bool remote_transmission_request) -> void {
    this->on_frame(can_id, remote_transmission_request, x);
#ifdef USE_MQTT
    this->mqtt_send_frame(can_id, x);
#endif
  };
  lambdaaction = new LambdaAction<std::vector<uint8_t>, uint32_t, bool>(cb);
  automation->add_actions({lambdaaction});
}

#ifdef USE_MQTT
void CanopenComponent::set_mqtt_client(esphome::mqtt::MQTTClientComponent *mqtt_client) {
  this->mqtt_client = mqtt_client;
  global_mqtt_client = mqtt_client;
  // mqtt::MQTTMessageTrigger *mqtt_message_trigger = new mqtt::MQTTMessageTrigger("raw-can-frame/#");
  CustomMQTTMessageTrigger *mqtt_message_trigger = new CustomMQTTMessageTrigger("canbus/#");
  mqtt_message_trigger->set_component_source("mqtt");
  App.register_component(mqtt_message_trigger);
  auto automation = new Automation<std::string, std::string>(mqtt_message_trigger);
  auto cb = [this](std::string topic, std::string payload) -> void {
    uint32_t can_id = 0xffffffff, src_can_id = 0xffffffff;
    sscanf(topic.c_str(), "canbus/%ld/%ld", &src_can_id, &can_id);
    bool is_own = (src_can_id < 2048) && (src_can_id & 0x7f) == this->node_id;
    ESP_LOGI(TAG, "received mqtt message on topic %s, len: %d, is_own: %d", topic.c_str(), payload.size(), is_own);
    if (is_own)
      return;
    auto data = std::vector<uint8_t>(payload.begin(), payload.end());
#ifdef USE_CANBUS
    if ((can_id & 0x7f) != this->node_id) {
      canbus->send_data(can_id, false, data);
    }
#endif
    this->on_frame(can_id, false, data);
    // this->on_frame(can_id, remote_transmission_request, x);
  };
  auto lambdaaction = new LambdaAction<std::string, std::string>(cb);
  automation->add_actions({lambdaaction});
}

void CanopenComponent::mqtt_send_frame(uint16_t addr, std::vector<uint8_t> data) {
  if (this->mqtt_client.has_value() && this->mqtt_client.value()->is_connected()) {
    std::stringstream topic;
    topic << "canbus/" << this->node_id << "/" << addr;
    this->mqtt_client.value()->publish(
        {.topic = topic.str(), .payload = std::string(data.begin(), data.end()), .qos = 0, .retain = false});
  }
}
#endif

uint32_t get_entity_index(uint32_t entity_id) { return 0x2000 + entity_id * 16; }

void CanopenComponent::od_add_metadata(uint32_t entity_id, uint8_t type, const std::string &name,
                                       const std::string &device_class, const std::string &unit,
                                       const std::string &state_class) {
  uint32_t index = get_entity_index(entity_id);
  ODAddUpdate(NodeSpec.Dict, CO_KEY(0x2000, entity_id, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA) type);
  if (name.size())
    ODAddUpdate(NodeSpec.Dict, CO_KEY(index, ENTITY_INDEX_NAME, CO_OBJ_____R_), CO_TSTRING, (CO_DATA) od_string(name));
  if (device_class.size())
    ODAddUpdate(NodeSpec.Dict, CO_KEY(index, ENTITY_INDEX_DEVICE_CLASS, CO_OBJ_____R_), CO_TSTRING,
                (CO_DATA) od_string(device_class));
  if (unit.size())
    ODAddUpdate(NodeSpec.Dict, CO_KEY(index, ENTITY_INDEX_UNIT, CO_OBJ_____R_), CO_TSTRING, (CO_DATA) od_string(unit));
  if (state_class.size())
    ODAddUpdate(NodeSpec.Dict, CO_KEY(index, ENTITY_INDEX_STATE_CLASS, CO_OBJ_D___R_), CO_TSTRING,
                (CO_DATA) od_string(state_class));
}

void CanopenComponent::od_add_sensor_metadata(uint32_t entity_id, float min_value, float max_value) {
  uint32_t index = get_entity_index(entity_id);
  // temporary pointers to get rid of aliasing warning
  uint32_t *min_value_ptr = (uint32_t *) &min_value;
  uint32_t *max_value_ptr = (uint32_t *) &max_value;
  ODAddUpdate(NodeSpec.Dict, CO_KEY(index, ENTITY_INDEX_SENSOR_MIN_VALUE, CO_OBJ_D___R_), CO_TUNSIGNED32,
              (CO_DATA) *min_value_ptr);
  ODAddUpdate(NodeSpec.Dict, CO_KEY(index, ENTITY_INDEX_SENSOR_MAX_VALUE, CO_OBJ_D___R_), CO_TUNSIGNED32,
              (CO_DATA) *max_value_ptr);
}

uint32_t CanopenComponent::od_add_state(uint32_t entity_id, const CO_OBJ_TYPE *type, void *state, uint8_t size,
                                        int8_t tpdo) {
  uint32_t async_pdo_mask = tpdo >= 0 ? CO_OBJ___A___ | CO_OBJ____P__ : 0;
  uint32_t entity_index = get_entity_index(entity_id);

  uint8_t state_sub_index = 0;
  auto obj = ODFind(NodeSpec.Dict, CO_DEV(entity_index + 1, 0));
  if (obj)
    state_sub_index = obj->Data;
  state_sub_index += 1;

  uint32_t value = 0;
  if (state && size)
    memcpy(&value, state, size);
  ODAddUpdate(NodeSpec.Dict, CO_KEY(entity_index + 1, state_sub_index, async_pdo_mask | CO_OBJ_D___R_), type, value);

  if (tpdo >= 0) {
    ODAddUpdate(NodeSpec.Dict, CO_KEY(0x1800 + tpdo, 1, CO_OBJ_DN__R_), CO_TUNSIGNED32,
                tpdo < 4 ? CO_COBID_TPDO_DEFAULT(tpdo) : CO_COBID_TPDO_DEFAULT(tpdo - 4) + 0x80);
    ODAddUpdate(NodeSpec.Dict, CO_KEY(0x1800 + tpdo, 2, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA) 254);

    uint8_t tpdo_sub_index = 0;
    auto obj = ODFind(NodeSpec.Dict, CO_DEV(0x1a00 + tpdo, 0));
    if (obj)
      tpdo_sub_index = obj->Data;
    tpdo_sub_index += 1;
    uint32_t bits = size * 8;
    ODAddUpdate(NodeSpec.Dict, CO_KEY(0x1a00 + tpdo, tpdo_sub_index, CO_OBJ_D___R_), CO_TUNSIGNED32,
                CO_LINK(entity_index + 1, state_sub_index, bits));
  }
  return CO_KEY(entity_index + 1, state_sub_index, 0);
}

void CanopenComponent::od_set_state(uint32_t key, void *state, uint8_t size) {
  auto obj = CODictFind(&node.Dict, key);
  if (!obj)
    return;
  if (!size) {
    size = obj->Type->Size(obj, &node, 4);
  }
  auto it = queued_states.begin();
  while (it != queued_states.end() && it->obj != obj)
    it++;
  if (it != queued_states.end()) {
    it->size = size;
    memcpy(&it->state, state, size);
    gettimeofday(&it->time, NULL);
  } else {
    QueuedState qstate = {obj, {}, size, {}};
    qstate.size = size;
    memcpy(&qstate.state, state, size);
    gettimeofday(&qstate.time, NULL);
    queued_states.push_back(qstate);
  };
}

uint32_t CanopenComponent::od_add_cmd(uint32_t entity_id, std::function<void(void *, uint32_t)> cb,
                                      const CO_OBJ_TYPE *type) {
  uint32_t index = get_entity_index(entity_id);

  uint8_t max_index = 0;
  auto obj = ODFind(NodeSpec.Dict, CO_DEV(index + 2, 0));
  if (obj)
    max_index = obj->Data;
  max_index += 1;

  ODAddUpdate(NodeSpec.Dict, CO_KEY(index + 2, max_index, CO_OBJ_D___RW), type, (CO_DATA) 0);
  auto key = CO_KEY(index + 2, max_index, 0);
  can_cmd_handlers[key] = cb;
  return key;
}

void CanopenComponent::rpdo_map_append(uint8_t idx, uint32_t index, uint8_t sub, uint8_t bit_size) {
  auto obj = ODFind(NodeSpec.Dict, CO_DEV(0x1600 + idx, 0));
  if (!obj) {
    ESP_LOGE(TAG, "RPDO map %d not found", idx);
    return;
  }
  uint8_t *sub_index_ptr = (uint8_t *) (obj->Data);
  *sub_index_ptr += 1;
  *(uint32_t *) (obj[*sub_index_ptr].Data) = CO_LINK(index, sub, bit_size);
}

void CanopenComponent::add_rpdo_dummy(uint8_t idx, uint8_t size) {
  while (size > 4) {
    add_rpdo_dummy(idx, 4);
    size -= 4;
  }
  switch (size) {
    case 1:
      rpdo_map_append(idx, 2, 0, size * 8);  // 2 or 5
      break;
    case 2:
      rpdo_map_append(idx, 3, 0, size * 8);  // 3 or 6
      break;
    case 3:
      add_rpdo_dummy(idx, 2);
      add_rpdo_dummy(idx, 1);
      break;
    case 4:
      rpdo_map_append(idx, 4, 0, size * 8);  // 4 or 7
      break;
  }
}
void CanopenComponent::add_rpdo_node(uint8_t idx, uint8_t node_id, uint8_t tpdo) {
  auto obj = ODFind(NodeSpec.Dict, CO_DEV(0x1400 + idx, 0));
  if (!obj) {
    ESP_LOGE(TAG, "RPDO param %d not found", idx);
    return;
  }
  *(uint8_t *) (obj->Data) = 2;
  *(uint32_t *) (obj[1].Data) =
      (tpdo < 4 ? CO_COBID_TPDO_DEFAULT(tpdo) : CO_COBID_TPDO_DEFAULT(tpdo - 4) + 0x80) + node_id;
  *(uint8_t *) (obj[2].Data) = 254;
}

void CanopenComponent::add_rpdo_entity_cmd(uint8_t idx, uint8_t entity_id, uint8_t cmd) {
  uint32_t index = get_entity_index(entity_id);
  rpdo_map_append(idx, index + 2, cmd + 1, 8);
}

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
void CanopenComponent::add_entity(sensor::Sensor *sensor, uint32_t entity_id, int8_t tpdo, uint8_t size, float min_val,
                                  float max_val) {
  od_add_metadata(entity_id,
                  size == 1   ? ENTITY_TYPE_SENSOR_UINT8
                  : size == 2 ? ENTITY_TYPE_SENSOR_UINT16
                              : ENTITY_TYPE_SENSOR,
                  sensor->get_name(), sensor->get_device_class(), sensor->get_unit_of_measurement(),
                  esphome::sensor::state_class_to_string(sensor->get_state_class()));
  od_add_sensor_metadata(entity_id, min_val, max_val);
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
  state_key = od_add_state(entity_id, type, &casted_state, size, tpdo);

  sensor->add_on_state_callback([=](float value) {
    auto casted_state = to_wire(value);
    od_set_state(state_key, &casted_state, size);
  });
  od_add_cmd(
      entity_id, [=](void *buffer, uint32_t size) { sensor->publish_state(from_wire(buffer)); }, cmd_type);
}
#endif

#ifdef USE_NUMBER
void CanopenComponent::add_entity(esphome::number::Number *number, uint32_t entity_id, int8_t tpdo, uint8_t size,
                                  float min_val, float max_val) {
  float state = number->state;
  od_add_metadata(entity_id,
                  size == 1   ? ENTITY_TYPE_NUMBER_UINT8
                  : size == 2 ? ENTITY_TYPE_NUMBER_UINT16
                              : ENTITY_TYPE_NUMBER,
                  number->get_name(), number->traits.get_device_class(), number->get_unit_of_measurement(), "");

  od_add_sensor_metadata(entity_id, min_val, max_val);
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
  state_key = od_add_state(entity_id, type, &casted_state, size, tpdo);
  number->add_on_state_callback([=](float value) {
    auto casted_state = to_wire(value);
    od_set_state(state_key, &casted_state, size);
  });
  od_add_cmd(
      entity_id, [=](void *buffer, uint32_t size) { number->publish_state(from_wire(buffer)); }, cmd_type);
}
#endif

#ifdef USE_BINARY_SENSOR
void CanopenComponent::add_entity(binary_sensor::BinarySensor *sensor, uint32_t entity_id, int8_t tpdo) {
  od_add_metadata(entity_id, ENTITY_TYPE_BINARY_SENSOR, sensor->get_name(), sensor->get_device_class(), "", "");
  auto state_key = od_add_state(entity_id, CO_TUNSIGNED8, &sensor->state, 1, tpdo);
  sensor->add_on_state_callback([=](bool x) { od_set_state(state_key, &x, 1); });
}
#endif

#ifdef USE_SWITCH
void CanopenComponent::add_entity(esphome::switch_::Switch *switch_, uint32_t entity_id, int8_t tpdo) {
  auto state = switch_->get_initial_state_with_restore_mode().value_or(false);
  od_add_metadata(entity_id, ENTITY_TYPE_SWITCH, switch_->get_name(), switch_->get_device_class(), "", "");
  auto state_key = od_add_state(entity_id, CO_TUNSIGNED8, &state, 1, tpdo);
  switch_->add_on_state_callback([=](bool value) { od_set_state(state_key, &value, 1); });
  od_add_cmd(entity_id, [=](void *buffer, uint32_t size) {
    if (((uint8_t *) buffer)[0]) {
      switch_->turn_on();
    } else {
      switch_->turn_off();
    }
  });
}
#endif

#ifdef USE_LIGHT
void CanopenComponent::add_entity(esphome::light::LightState *light, uint32_t entity_id, int8_t tpdo) {
  bool state = bool(light->remote_values.get_state());
  uint8_t brightness = (uint8_t) (light->remote_values.get_brightness() * 255);
  uint16_t colortemp = (uint16_t) (light->remote_values.get_color_temperature());
  od_add_metadata(entity_id, ENTITY_TYPE_LIGHT, light->get_name(), "", "", "");
  auto state_key = od_add_state(entity_id, CO_TUNSIGNED8, &state, 1, tpdo);
  auto brightness_key = od_add_state(entity_id, CO_TUNSIGNED8, &brightness, 1, tpdo);
  auto colortemp_key = od_add_state(entity_id, CO_TUNSIGNED16, &colortemp, 2, tpdo);
  light->add_new_remote_values_callback([=]() {
    bool value = bool(light->remote_values.get_state());
    uint8_t brightness = (uint8_t) (light->remote_values.get_brightness() * 255);
    uint16_t colortemp = (uint16_t) (light->remote_values.get_color_temperature());
    od_set_state(state_key, &value, 1);
    od_set_state(brightness_key, &brightness, 1);
    od_set_state(colortemp_key, &colortemp, 2);
  });
  od_add_cmd(entity_id,
             [=](void *buffer, uint32_t size) { light->make_call().set_state(((uint8_t *) buffer)[0]).perform(); });
  od_add_cmd(entity_id, [=](void *buffer, uint32_t size) {
    light->make_call().set_brightness_if_supported(float(((uint8_t *) buffer)[0]) / 255.0).perform();
  });
  od_add_cmd(
      entity_id,
      [=](void *buffer, uint32_t size) {
        light->make_call().set_color_temperature_if_supported(float(((uint16_t *) buffer)[0])).perform();
      },
      CO_TCMD16);
}
#endif

void CanopenComponent::add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<uint8_t> *trigger) {
  od_add_cmd(
      entity_id, [=](void *buffer, uint32_t size) { trigger->trigger(*((uint8_t *) buffer)); }, CO_TCMD8);
}

void CanopenComponent::add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<int8_t> *trigger) {
  od_add_cmd(
      entity_id, [=](void *buffer, uint32_t size) { trigger->trigger(*((int8_t *) buffer)); }, CO_TCMD8);
}

void CanopenComponent::add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<uint16_t> *trigger) {
  od_add_cmd(
      entity_id, [=](void *buffer, uint32_t size) { trigger->trigger(*((uint16_t *) buffer)); }, CO_TCMD16);
}

void CanopenComponent::add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<int16_t> *trigger) {
  od_add_cmd(
      entity_id, [=](void *buffer, uint32_t size) { trigger->trigger(*((int16_t *) buffer)); }, CO_TCMD16);
}

void CanopenComponent::add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<uint32_t> *trigger) {
  od_add_cmd(
      entity_id, [=](void *buffer, uint32_t size) { trigger->trigger(*((uint32_t *) buffer)); }, CO_TCMD32);
}

void CanopenComponent::add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<int32_t> *trigger) {
  od_add_cmd(
      entity_id, [=](void *buffer, uint32_t size) { trigger->trigger(*((int32_t *) buffer)); }, CO_TCMD32);
}

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

void CanopenComponent::add_entity(esphome::cover::Cover *cover, uint32_t entity_id, int8_t tpdo) {
  uint8_t state = get_cover_state(cover);
  uint8_t position = uint8_t(cover->position * 255);
  od_add_metadata(entity_id, ENTITY_TYPE_COVER, cover->get_name(), cover->get_device_class(), "", "");
  auto state_key = od_add_state(entity_id, CO_TUNSIGNED8, &state, 1, tpdo);
  auto pos_key = od_add_state(entity_id, CO_TUNSIGNED8, &position, 1, tpdo);
  cover->add_on_state_callback([=]() {
    ESP_LOGD(TAG, "on_state callback, op: %s, pos: %f", cover_operation_to_str(cover->current_operation),
             cover->position);
    uint8_t position = uint8_t(cover->position * 255);
    uint8_t state = get_cover_state(cover);
    od_set_state(state_key, &state, 1);
    od_set_state(pos_key, &position, 1);
  });
  od_add_cmd(entity_id, [=](void *buffer, uint32_t size) {
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
  od_add_cmd(entity_id, [=](void *buffer, uint32_t size) {
    uint8_t cmd = *(uint8_t *) buffer;
    float position = ((float) cmd) / 255.0;
    auto call = cover->make_call();
    call.set_position(position);
    call.perform();
  });
}
#endif

// void CanopenComponent::add_status(uint32_t entity_id, uint32_t update_interval) {
//   struct timeval tv_now;
//   gettimeofday(&tv_now, NULL);
//   tv_now.tv_sec -= (time_t)(esp_random() % update_interval);
//   status = Status {
//     entity_id: entity_id,
//     last_time: tv_now,
//     update_interval: update_interval
//   };

//   configure_entity([=]() {
//     uint8_t device_class = 9; // data-size
//     std::vector<uint8_t> data = { ENTITY_TYPE_CAN_STATUS, device_class, 0 };
//     canbus->send_data((entity_id << 4) | PROPERTY_CONFIG, true, data);
//     can_send_string_prop(entity_id, PROPERTY_CONFIG_NAME, App.get_name());
//   });
// }

const char *SensorName = "Sensor1";
const char *SensorUnit = "deg";

CO_OBJ_STR ManufacturerDeviceNameObj = {0, (uint8_t *) "ESPHome"};

#ifdef USE_CANOPEN_OTA
Firmware FirmwareObj;
uint8_t FirmwareMD5Data[32];

CO_OBJ_DOM FirmwareMD5 = {
    0,
    sizeof(FirmwareObj.md5),
    FirmwareObj.md5,
};
#endif

void CanopenComponent::setup_csdo(uint8_t num, uint8_t node_id, uint32_t tx_id, uint32_t rx_id) {
  ODAddUpdate(NodeSpec.Dict, CO_KEY(0x1280 + num, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, 0);
  ODAddUpdate(NodeSpec.Dict, CO_KEY(0x1280 + num, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, tx_id);
  ODAddUpdate(NodeSpec.Dict, CO_KEY(0x1280 + num, 2, CO_OBJ_D___R_), CO_TUNSIGNED32, rx_id);
  ODAddUpdate(NodeSpec.Dict, CO_KEY(0x1280 + num, 3, CO_OBJ_D___R_), CO_TUNSIGNED8, node_id);
}

void CanopenComponent::od_set_string(uint32_t index, uint32_t sub, const char *value) {
  ODAddUpdate(NodeSpec.Dict, CO_KEY(index, sub, CO_OBJ_____R_), CO_TSTRING, (CO_DATA) od_string(value));
}

void CanopenComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CANopen...");
  ESP_LOGD(TAG, "CO_TPDO_N: %d", CO_TPDO_N);
  ESP_LOGD(TAG, "CO_RPDO_N: %d", CO_RPDO_N);

  uint32_t hash = fnv1_hash("canopen_comm_state_v2");
  this->comm_state = global_preferences->make_preference<uint8_t[CO_RPDO_N * 41]>(hash, true);
  if (this->comm_state.load(&rpdo_buf)) {
    ESP_LOGI(TAG, "loaded RPDO config from preferences");
  } else {
    ESP_LOGI(TAG, "can't load RPDO config from preferences, using defaults");
  }

  if (heartbeat_interval_ms) {
    ODAddUpdate(NodeSpec.Dict, CO_KEY(0x1017, 0, CO_OBJ_D___RW), CO_THB_PROD, (CO_DATA) (heartbeat_interval_ms));
  }

  // manufacturer device name
  od_set_string(0x1008, 0, App.get_name().c_str());

  for (uint8_t i = 0; i < 8; i++) {
    ODAddUpdate(NodeSpec.Dict, CO_KEY(0x1800 + i, 1, CO_OBJ_DN__R_), CO_TUNSIGNED32,
                i < 4 ? CO_COBID_TPDO_DEFAULT(i) : CO_COBID_TPDO_DEFAULT(i - 4) + 0x80);
    ODAddUpdate(NodeSpec.Dict, CO_KEY(0x1800 + i, 2, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA) 254);
  }

#ifdef USE_CANOPEN_OTA
  FirmwareObj.domain.Size = 1024 * 1024;

  ODAddUpdate(NodeSpec.Dict, CO_KEY(0x3000, 1, CO_OBJ_D____W), FW_CTRL, (CO_DATA) 0);
  ODAddUpdate(NodeSpec.Dict, CO_KEY(0x3000, 2, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA) (&FirmwareObj.size));
  ODAddUpdate(NodeSpec.Dict, CO_KEY(0x3000, 3, CO_OBJ______W), CO_TDOMAIN, (CO_DATA) (&FirmwareMD5));
  ODAddUpdate(NodeSpec.Dict, CO_KEY(0x3000, 4, CO_OBJ______W), FW_IMAGE, (CO_DATA) (&FirmwareObj));
#endif

  CONodeInit(&node, &NodeSpec);
  auto err = CONodeGetErr(&node);
  if (err != CO_ERR_NONE) {
    ESP_LOGE(TAG, "canopen init error: %d", err);
  }

  CONodeStart(&node);
  set_pre_operational_mode();
}

void CanopenComponent::set_pre_operational_mode() {
  CONmtSetMode(&node.Nmt, CO_PREOP);
  ESP_LOGI(TAG, "node is pre_operational");
  if (on_pre_operational) {
    on_pre_operational->trigger();
  } else {
    set_operational_mode();
  }
}

void CanopenComponent::set_operational_mode() {
  // update dictionary size
  // as new entries may have been added on pre_operational phase
  CODictInit(&node.Dict, &node, NodeSpec.Dict, NodeSpec.DictLen);
  CONmtSetMode(&node.Nmt, CO_OPERATIONAL);

  ESP_LOGD(TAG, "############# Object Dictionary #############");
  uint16_t index = 0;
  for (auto od = node.Dict.Root; index < NodeSpec.DictLen && (od->Key != 0);) {
    uint32_t value = od->Key & CO_OBJ_D_____ ? od->Data : (*(uint32_t *) od->Data);
    if (od->Type && od->Type->Size) {
      auto size = od->Type->Size(od, &node, 4);
      if (size == 1)
        value &= 0xff;
      else if (size == 2)
        value &= 0xffff;
    }
    ESP_LOGD(TAG, "OD Index: %02x Key: %08lx Data: %08lx Type: %p", index, od->Key, value, od->Type);
    index++;
    od++;
  }
  ESP_LOGD(TAG, "#############################################");

  ESP_LOGI(TAG, "node is operational");

  if (on_operational)
    on_operational->trigger();
}

void CanopenComponent::trig_tpdo(int8_t num) {
  for (auto tpdo = 0; tpdo < CO_TPDO_N; tpdo++) {
    if (node.TPdo[tpdo].ObjNum > 0 && (num < 0 || tpdo == num)) {
      COTPdoTrigPdo(node.TPdo, tpdo);
    }
  }
}

void CanopenComponent::csdo_recv(uint8_t num, uint32_t key, std::function<void(uint32_t, uint32_t)> cb) {
  static auto csdo0 = COCSdoFind(&node, 0);
  if (!csdo0) {
    return cb(0, -1);
  }
  static uint32_t buffers[CO_CSDO_N];
  static std::function<void(uint32_t, uint32_t)> callbacks[CO_CSDO_N];
  auto csdo = COCSdoFind(&node, num);
  if (csdo) {
    buffers[num] = 0;
    callbacks[num] = cb;
    auto ret = COCSdoRequestUpload(
        csdo, key, (uint8_t *) &buffers[num], 4,
        [](CO_CSDO_T *csdo, uint16_t index, uint8_t sub, uint32_t code) {
          auto num = csdo - csdo0;
          ESP_LOGV(TAG, "COCSdoRequestUpload cb: %04x %02x %08x", index, sub, code);
          callbacks[num](buffers[num], code);
        },
        1000);
  } else {
    cb(0, -1);
  }
}

void CanopenComponent::csdo_send_data(uint8_t num, uint32_t key, uint8_t *data, uint8_t len) {
  auto csdo = COCSdoFind(&node, num);
  if (csdo) {
    auto ret = COCSdoRequestDownload(
        csdo, key, data, len,
        [](CO_CSDO_T *csdo, uint16_t index, uint8_t sub, uint32_t code) {
          ESP_LOGV(TAG, "COCSdoRequestDownload cb: %04x %02x %08x", index, sub, code);
        },
        1000);

    if (ret) {
      ESP_LOGE(TAG, "COCSdoRequestDownload err: %08x", ret);
    }
  }
}

bool CanopenComponent::get_can_status(CanStatus &status_info) {
  twai_status_info_t twai_status_info;
  if (twai_get_status_info(&twai_status_info) == ESP_OK) {
    status_info.state = twai_status_info.state;
    status_info.bus_err = twai_status_info.bus_error_count;
    status_info.arb_lost = twai_status_info.arb_lost_count;
    status_info.tx_err = twai_status_info.tx_error_counter;
    status_info.rx_err = twai_status_info.rx_error_counter;
    status_info.tx_failed = twai_status_info.tx_failed_count;
    status_info.rx_miss = twai_status_info.rx_missed_count;
    return true;
  }
  return false;
}

void CanopenComponent::initiate_recovery() { twai_initiate_recovery(); }

void CanopenComponent::start() { twai_start(); }

void CanopenComponent::store_comm_params() {
  if (comm_state.save(&rpdo_buf)) {
    global_preferences->sync();
    ESP_LOGI(TAG, "Stored communication params in NVM");
  } else {
    ESP_LOGE(TAG, "Can't store communication params in NVM");
  }
}

void CanopenComponent::reset_comm_params() {
  global_preferences->reset();
  ESP_LOGI(TAG, "Reseted communication params in NVM");
}

int16_t CanopenComponent::get_heartbeat_events(uint8_t node_id) { return CONmtGetHbEvents(&node.Nmt, node_id); }

void CanopenComponent::setup_heartbeat_client(uint8_t subidx, uint8_t node_id, uint16_t timeout_ms) {
  CO_HBCONS *thb_cons = new CO_HBCONS;
  memset(thb_cons, 0, sizeof(CO_HBCONS));
  thb_cons->Time = timeout_ms;
  thb_cons->NodeId = node_id;
  ODAddUpdate(NodeSpec.Dict, CO_KEY(0x1016, subidx, CO_OBJ_____RW), CO_THB_CONS, (CO_DATA) thb_cons);
}

void CanopenComponent::loop() {
  COTmrService(&node.Tmr);
  COTmrProcess(&node.Tmr);

  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);

  for (auto it = queued_states.begin(); it != queued_states.end();) {
    uint32_t diff = (tv_now.tv_sec - it->time.tv_sec) * 1000000 + (tv_now.tv_usec - it->time.tv_usec);
    if (diff >= state_update_delay_us) {
      COObjWrValue(it->obj, &node, &it->state, it->size);
      it = queued_states.erase(it);
    } else {
      ++it;
    }
  }

  if (abs(tv_now.tv_sec - status_time.tv_sec) >= status_update_interval) {
    if (get_can_status(status)) {
      if (status.tx_err > last_status.tx_err || status.rx_err > last_status.rx_err ||
          status.tx_failed > last_status.tx_failed || status.rx_miss > last_status.rx_miss ||
          status.arb_lost > last_status.arb_lost || status.bus_err > last_status.bus_err) {
        ESP_LOGW(TAG, "tx_err: ld rx_err: %ld tx_failed: %ld rx_miss: %ld arb_lost: %ld bus_err: %ld", status.tx_err,
                 status.rx_err, status.tx_failed, status.rx_miss, status.arb_lost, status.bus_err);
      }
      last_status = status;
    }
    status_time = tv_now;
  }
}
}  // namespace canopen
}  // namespace esphome

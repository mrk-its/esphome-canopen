#pragma once

#include "esphome.h"
using namespace esphome;

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
// #include "esphome/components/canbus/canbus.h"
#ifdef USE_MQTT
#include "esphome/components/mqtt/mqtt_client.h"
#endif
#include <vector>
#include <map>
#include <driver/twai.h>
#include <sstream>

#include "node_spec.h"
#include "co_if.h"
#include "co_cmd.h"

#ifdef USE_SENSOR
namespace esphome {
namespace sensor {
class Sensor;
}
}  // namespace esphome
#endif

#ifdef USE_NUMBER
namespace esphome {
namespace number {
class Number;
}
}  // namespace esphome
#endif

#ifdef USE_BINARY_SENSOR
namespace esphome {
namespace binary_sensor {
class BinarySensor;
}
}  // namespace esphome
#endif

#ifdef USE_SWITCH
namespace esphome {
namespace switch_ {
class Switch;
}
}  // namespace esphome
#endif

#ifdef USE_LIGHT
namespace esphome {
namespace light {
class LightState;
}
}  // namespace esphome
#endif

#ifdef USE_COVER
namespace esphome {
namespace cover {
class Cover;
}
}  // namespace esphome
#endif

const int8_t ENTITY_TYPE_DISABLED = 0;
const int8_t ENTITY_TYPE_SENSOR = 1;
const int8_t ENTITY_TYPE_BINARY_SENSOR = 2;
const int8_t ENTITY_TYPE_SWITCH = 3;
const int8_t ENTITY_TYPE_COVER = 4;
const int8_t ENTITY_TYPE_LIGHT = 5;
const int8_t ENTITY_TYPE_SENSOR_UINT8 = 6;
const int8_t ENTITY_TYPE_SENSOR_UINT16 = 7;
const int8_t ENTITY_TYPE_NUMBER = 8;
const int8_t ENTITY_TYPE_NUMBER_UINT8 = 9;
const int8_t ENTITY_TYPE_NUMBER_UINT16 = 10;
const uint8_t ENTITY_TYPE_CAN_STATUS = 254;

const uint8_t ENTITY_INDEX_NAME = 1;
const uint8_t ENTITY_INDEX_DEVICE_CLASS = 2;
const uint8_t ENTITY_INDEX_UNIT = 3;
const uint8_t ENTITY_INDEX_STATE_CLASS = 4;

const uint8_t ENTITY_INDEX_SENSOR_MIN_VALUE = 7;
const uint8_t ENTITY_INDEX_SENSOR_MAX_VALUE = 8;

const int32_t PROPERTY_STATE0 = 0;
const int32_t PROPERTY_STATE1 = 1;
const int32_t PROPERTY_STATE2 = 2;
const int32_t PROPERTY_STATE3 = 3;

const int32_t PROPERTY_CMD0 = 4;
const int32_t PROPERTY_CMD1 = 5;
const int32_t PROPERTY_CMD2 = 6;
const int32_t PROPERTY_CMD3 = 7;

const int32_t PROPERTY_CONFIG = 8;  // D0 - entity type, D1: device_class, D2: state_class

const int32_t PROPERTY_CONFIG_NAME = 9;
const int32_t PROPERTY_CONFIG_NAME2 = 10;
const int32_t PROPERTY_CONFIG_UNIT = 11;

static const char *const TAG = "canopen";

#define ENTITY_STATE_KEY(entity_id, state_num) (CO_KEY(0x2000 + entity_id * 16 + 1, state_num + 1, 0))
#define ENTITY_CMD_KEY(entity_id, cmd_num) (CO_KEY(0x2000 + entity_id * 16 + 2, cmd_num + 1, 0))

namespace esphome {
namespace canopen {
struct CanStatus {
  uint8_t state;
  uint32_t tx_err;
  uint32_t rx_err;
  uint32_t tx_failed;
  uint32_t rx_miss;
  uint32_t arb_lost;
  uint32_t bus_err;
};

struct QueuedState {
  CO_OBJ_T *obj;
  uint32_t state;
  uint8_t size;
  struct timeval time;
};

const uint32_t status_update_interval = 1;

class OperationalTrigger : public Trigger<> {};
class PreOperationalTrigger : public Trigger<> {};
class HbConsumerEventTrigger : public Trigger<uint8_t> {};

class CmdTriggerUInt8 : public Trigger<uint8_t> {};
class CmdTriggerUInt16 : public Trigger<uint16_t> {};
class CmdTriggerUInt32 : public Trigger<uint32_t> {};

class CmdTriggerInt8 : public Trigger<int8_t> {};
class CmdTriggerInt16 : public Trigger<int16_t> {};
class CmdTriggerInt32 : public Trigger<int32_t> {};

class CanopenComponent : public Component {
  CO_NODE node;
  uint32_t node_id;
  CanStatus last_status;
  struct timeval status_time;

  void rpdo_map_append(uint8_t idx, uint32_t index, uint8_t sub, uint8_t size);

  OperationalTrigger *on_operational;
  PreOperationalTrigger *on_pre_operational;
  std::vector<QueuedState> queued_states;
  uint32_t state_update_delay_us;
  uint16_t heartbeat_interval_ms;

  ESPPreferenceObject comm_state;

 public:
  HbConsumerEventTrigger *on_hb_cons_event;
  CanStatus status;
  std::map<uint32_t, std::function<void(void *, uint32_t)>> can_cmd_handlers;

  optional<CO_IF_FRM> recv_frame;

#ifdef USE_CANBUS
  canbus::Canbus *canbus;
  void set_canbus(canbus::Canbus *canbus);
#endif

#ifdef USE_MQTT
  optional<esphome::mqtt::MQTTClientComponent *> mqtt_client;

  void set_mqtt_client(esphome::mqtt::MQTTClientComponent *mqtt_client);

  void mqtt_send_frame(uint16_t addr, std::vector<uint8_t> data);
#endif

  CanopenComponent(uint32_t node_id);

  void add_trigger(OperationalTrigger *trigger) { on_operational = trigger; }
  void add_trigger(PreOperationalTrigger *trigger) { on_pre_operational = trigger; }
  void add_trigger(HbConsumerEventTrigger *trigger) { on_hb_cons_event = trigger; }
  void set_pre_operational_mode();
  void set_operational_mode();

  void trig_tpdo(int8_t num = -1);

  void setup_csdo(uint8_t num, uint8_t node_id, uint32_t tx_id, uint32_t rx_id);
  void csdo_recv(uint8_t num, uint32_t key, std::function<void(uint32_t, uint32_t)> cb);

  void csdo_send_data(uint8_t num, uint32_t key, uint8_t *data, uint8_t len);
  void csdo_send_u8(uint8_t num, uint32_t key, uint8_t value) { csdo_send_data(num, key, (uint8_t *) (&value), 1); }
  void csdo_send_u16(uint8_t num, uint32_t key, uint16_t value) { csdo_send_data(num, key, (uint8_t *) (&value), 2); }
  void csdo_send_u32(uint8_t num, uint32_t key, uint32_t value) { csdo_send_data(num, key, (uint8_t *) (&value), 4); }
  void csdo_send_float(uint8_t num, uint32_t key, float value) { csdo_send_data(num, key, (uint8_t *) (&value), 4); }

  float get_setup_priority() const override { return setup_priority::HARDWARE - 1.0; }
  void setup();
  bool is_initialized() { return node.Nmt.Mode == CO_PREOP || node.Nmt.Mode == CO_OPERATIONAL; }
  void add_rpdo_dummy(uint8_t idx, uint8_t size);
  void add_rpdo_node(uint8_t idx, uint8_t node_id, uint8_t tpdo);
  void add_rpdo_entity_cmd(uint8_t idx, uint8_t entity_id, uint8_t cmd);

  void add_entity(EntityBase *entity, uint32_t entity_id, int8_t tpdo, uint8_t size = 4, float min_val = 0,
                  float max_val = 0);

  void add_entity(EntityBase *sensor, uint32_t entity_id, int8_t tpdo);

// void add_status(uint32_t entity_id, uint32_t update_interval);
#ifdef USE_SENSOR
  void add_entity(sensor::Sensor *sensor, uint32_t entity_id, int8_t tpdo, uint8_t size = 4, float min_val = 0,
                  float max_val = 0);
#endif

#ifdef USE_NUMBER
  void add_entity(esphome::number::Number *number, uint32_t entity_id, int8_t tpdo, uint8_t size = 4, float min_val = 0,
                  float max_val = 0);
#endif

#ifdef USE_BINARY_SENSOR
  void add_entity(binary_sensor::BinarySensor *sensor, uint32_t entity_id, int8_t tpdo);
#endif

#ifdef USE_SWITCH
  void add_entity(esphome::switch_::Switch *switch_, uint32_t entity_id, int8_t tpdo);
#endif

#ifdef USE_LIGHT
  void add_entity(esphome::light::LightState *light, uint32_t entity_id, int8_t tpdo);
#endif

#ifdef USE_COVER
  void add_entity(esphome::cover::Cover *cover, uint32_t entity_id, int8_t tpdo);
#endif

  void od_add_metadata(uint32_t entity_id, uint8_t type, const std::string &name, const std::string &device_class,
                       const std::string &unit, const std::string &state_class);
  void od_add_sensor_metadata(uint32_t entity_id, float min_value, float max_value);
  uint32_t od_add_state(uint32_t entity_id, const CO_OBJ_TYPE *type, void *state, uint8_t size, int8_t tpdo);
  uint32_t od_add_cmd(uint32_t entity_id, std::function<void(void *, uint32_t)> cb, const CO_OBJ_TYPE *type = CO_TCMD8);

  void add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<uint8_t> *trigger);
  void add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<int8_t> *trigger);
  void add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<uint16_t> *trigger);
  void add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<int16_t> *trigger);
  void add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<uint32_t> *trigger);
  void add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<int32_t> *trigger);

  void od_set_string(uint32_t index, uint32_t sub, const char *value);
  void set_state_update_delay(uint32_t delay_ms);
  void set_heartbeat_interval(uint16_t interval_ms);
  void setup_heartbeat_client(uint8_t subidx, uint8_t node_id, uint16_t timeout_ms);
  int16_t get_heartbeat_events(uint8_t node_id);
  void initiate_recovery();
  void start();
  void od_set_state(uint32_t key, void *state, uint8_t size);
  void set_entity_state(uint32_t entity_id, uint32_t state, void *data, uint8_t size) {
    od_set_state(ENTITY_STATE_KEY(entity_id, state), data, size);
  }
  void on_frame(uint32_t can_id, bool rtr, std::vector<uint8_t> &data);
  void store_comm_params();
  void reset_comm_params();
  void loop() override;
  bool get_can_status(CanStatus &status_info);
};
}  // namespace canopen
extern canopen::CanopenComponent *global_canopen;
}  // namespace esphome

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

#ifdef USE_CANOPEN_OTA
#include "ota/ota.h"
#endif

namespace esphome {

#ifdef USE_SENSOR
namespace sensor {
class Sensor;
}
#endif

#ifdef USE_NUMBER
namespace number {
class Number;
}
#endif

#ifdef USE_BINARY_SENSOR
namespace binary_sensor {
class BinarySensor;
}
#endif

#ifdef USE_SWITCH
namespace switch_ {
class Switch;
}
#endif

#ifdef USE_LIGHT
namespace light {
class LightState;
}
#endif

#ifdef USE_COVER
namespace cover {
class Cover;
}
#endif

#ifdef USE_ALARM_CONTROL_PANEL
namespace template_ {
class TemplateAlarmControlPanel;
}
#endif
}  // namespace esphome


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
const int8_t ENTITY_TYPE_ALARM = 16;

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

class CanopenComponent;

class BaseCanopenEntity {
  public:
  uint32_t entity_id;
  int8_t tpdo;
  BaseCanopenEntity(uint32_t entity_id, int8_t tpdo) {
    this->entity_id = entity_id;
    this->tpdo = tpdo;
  }
  virtual void setup(CanopenComponent *canopen) = 0;
};


#ifdef USE_SENSOR
  class SensorEntity: public BaseCanopenEntity {
    public:
    sensor::Sensor *sensor;
    uint8_t size;
    float min_val;
    float max_val;
    SensorEntity(sensor::Sensor *sensor, uint32_t entity_id, int8_t tpdo,
                 uint8_t size = 4, float min_val = 0, float max_val = 0) : BaseCanopenEntity(entity_id, tpdo) {
      this->sensor = sensor;
      this->size = size;
      this->min_val = min_val;
      this->max_val = max_val;
    }
    void setup(CanopenComponent *canopen) override;
  };
#endif

#ifdef USE_NUMBER
  class NumberEntity: public BaseCanopenEntity {
    public:
    esphome::number::Number *number;
    uint8_t size;
    float min_val;
    float max_val;
    NumberEntity(esphome::number::Number *number, uint32_t entity_id, int8_t tpdo,
                 uint8_t size = 4, float min_val = 0, float max_val = 0) : BaseCanopenEntity(entity_id, tpdo) {
      this->number = number;
      this->size = size;
      this->min_val = min_val;
      this->max_val = max_val;
    }
    void setup(CanopenComponent *canopen) override;
  };
#endif

#ifdef USE_BINARY_SENSOR
  class BinarySensorEntity: public BaseCanopenEntity {
    public:
    binary_sensor::BinarySensor *sensor;
    BinarySensorEntity(binary_sensor::BinarySensor *sensor, uint32_t entity_id, int8_t tpdo) : BaseCanopenEntity(entity_id, tpdo) {
      this->sensor = sensor;
    }
    void setup(CanopenComponent *canopen) override;
  };
#endif

#ifdef USE_SWITCH
  class SwitchEntity: public BaseCanopenEntity {
    public:
    esphome::switch_::Switch *switch_;
    SwitchEntity(esphome::switch_::Switch *switch_, uint32_t entity_id, int8_t tpdo) : BaseCanopenEntity(entity_id, tpdo) {
      this->switch_ = switch_;
    }
    void setup(CanopenComponent *canopen) override;
  };
#endif

#ifdef USE_LIGHT
  class LightStateEntity: public BaseCanopenEntity {
    public:
    esphome::light::LightState *light;
    LightStateEntity(esphome::light::LightState *light, uint32_t entity_id, int8_t tpdo) : BaseCanopenEntity(entity_id, tpdo) {
      this->light = light;
    }
    void setup(CanopenComponent *canopen) override;
  };
#endif

#ifdef USE_COVER
  class CoverEntity: public BaseCanopenEntity {
    public:
    esphome::cover::Cover *cover;
    CoverEntity(esphome::cover::Cover *cover, uint32_t entity_id, int8_t tpdo) : BaseCanopenEntity(entity_id, tpdo) {
      this->cover = cover;
    }
    void setup(CanopenComponent *canopen) override;
  };
#endif

#ifdef USE_ALARM_CONTROL_PANEL
  class AlarmEntity: public BaseCanopenEntity {
    public:
    esphome::template_::TemplateAlarmControlPanel *alarm;
    AlarmEntity(esphome::template_::TemplateAlarmControlPanel *alarm, uint32_t entity_id, int8_t tpdo) : BaseCanopenEntity(entity_id, tpdo) {
      this->alarm = alarm;
    }
    void setup(CanopenComponent *canopen) override;
  };
#endif


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
  std::vector<BaseCanopenEntity *> entities;

  uint32_t state_update_delay_us;
  uint16_t heartbeat_interval_ms;

  ESPPreferenceObject comm_state;

 public:
  HbConsumerEventTrigger *on_hb_cons_event;
  CanStatus status;
  std::map<uint32_t, std::function<void(void *, uint32_t)>> can_cmd_handlers;

  optional<CO_IF_FRM> recv_frame;

#ifdef USE_CANOPEN_OTA
  CanopenOTAComponent *ota;
  void set_ota(CanopenOTAComponent *ota) { this->ota = ota; }
#endif

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

  float get_setup_priority() const override { return setup_priority::LATE;}
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
                  float max_val = 0) {
    entities.push_back(new SensorEntity(sensor, entity_id, tpdo, size, min_val, max_val));
  }
                  
#endif

#ifdef USE_NUMBER
  void add_entity(esphome::number::Number *number, uint32_t entity_id, int8_t tpdo, uint8_t size = 4, float min_val = 0,
                  float max_val = 0) {
    entities.push_back(new NumberEntity(number, entity_id, tpdo, size, min_val, max_val));
  }
#endif

#ifdef USE_BINARY_SENSOR
  void add_entity(binary_sensor::BinarySensor *sensor, uint32_t entity_id, int8_t tpdo) {
    entities.push_back(new BinarySensorEntity(sensor, entity_id, tpdo));
  }
#endif

#ifdef USE_SWITCH
  void add_entity(esphome::switch_::Switch *switch_, uint32_t entity_id, int8_t tpdo) {
    entities.push_back(new SwitchEntity(switch_, entity_id, tpdo));
  }
#endif

#ifdef USE_LIGHT
  void add_entity(esphome::light::LightState *light, uint32_t entity_id, int8_t tpdo) {
    entities.push_back(new LightStateEntity(light, entity_id, tpdo));
  }
#endif

#ifdef USE_COVER
  void add_entity(esphome::cover::Cover *cover, uint32_t entity_id, int8_t tpdo) {
    entities.push_back(new CoverEntity(cover, entity_id, tpdo));
  }
#endif

#ifdef USE_ALARM_CONTROL_PANEL
  void add_entity(esphome::template_::TemplateAlarmControlPanel *alarm, uint32_t entity_id, int8_t tpdo) {
    entities.push_back(new AlarmEntity(alarm, entity_id, tpdo));
  }
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

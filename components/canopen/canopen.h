#pragma once

#include "entities.h"
#include "driver_can.h"
#include "od.h"

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

#define ENTITY_INDEX(entity_id) (0x2000 + entity_id * 16)
#define ENTITY_STATE_KEY(entity_id, state_num) (CO_KEY(ENTITY_INDEX(entity_id) + 1, state_num + 1, 0))
#define ENTITY_CMD_KEY(entity_id, cmd_num) (CO_KEY(ENTITY_INDEX(entity_id) + 2, cmd_num + 1, 0))

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

const uint32_t status_update_interval_ms = 5000;

class OperationalTrigger : public Trigger<> {};
class PreOperationalTrigger : public Trigger<> {};
class HbConsumerEventTrigger : public Trigger<uint8_t> {};

class CmdTriggerUInt8 : public Trigger<uint8_t> {};
class CmdTriggerUInt16 : public Trigger<uint16_t> {};
class CmdTriggerUInt32 : public Trigger<uint32_t> {};

class CmdTriggerInt8 : public Trigger<int8_t> {};
class CmdTriggerInt16 : public Trigger<int16_t> {};
class CmdTriggerInt32 : public Trigger<int32_t> {};

#define APP_BAUDRATE 125000u       /* CAN baudrate                */
#define APP_TMR_N 16u              /* Number of software timers   */
#define APP_TICKS_PER_SEC 1000000u /* Timer clock frequency in Hz */

#ifndef APP_OBJ_N
#define APP_OBJ_N 512u /* Object dictionary max size  */
#endif

enum EMCY_CODES {
  APP_ERR_ID_EEPROM = 0,
  APP_ERR_ID_NUM /* number of EMCY error codes in application */
};

class CanopenComponent;

struct CanopenNode {
  CO_NODE node;
  CanopenComponent *canopen;
};

class CanopenComponent : public Component {
 protected:
  // for timer driver
  uint64_t next_timer_us;

  /* Each software timer needs some memory for managing
   * the lists and states of the timed action events.
   */
  CO_TMR_MEM TmrMem[APP_TMR_N];

  /* Each SDO server needs memory for the segmented or
   * block transfer requests.
   */
  uint8_t SdoSrvMem[CO_SSDO_N * CO_SDO_BUF_BYTE];

  /* Specify the EMCY error codes with the corresponding
   * error register bit. There is a collection of defines
   * for the predefined emergency codes CO_EMCY_CODE...
   * and for the error register bits CO_EMCY_REG... for
   * readability. You can use plain numbers, too.
   */
  CO_EMCY_TBL AppEmcyTbl[APP_ERR_ID_NUM] = {
      {CO_EMCY_REG_GENERAL, CO_EMCY_CODE_HW_ERR} /* APP_ERR_ID_EEPROM */
  };

  uint8_t rpdo_buf[CO_RPDO_N][41];

  ObjectDictionary od;

  std::vector<CO_IF_FRM> recv_frames;
  friend class BaseCanopenEntity;

  friend int16_t esphome::canopen::DrvCanSend(CO_IF_FRM *frm);
  friend int16_t esphome::canopen::DrvCanRead(CO_IF_FRM *frm);

  friend void DrvTimerInit(uint32_t freq);
  friend void DrvTimerStart(void);
  friend uint8_t DrvTimerUpdate(void);
  friend uint32_t DrvTimerDelay(void);
  friend void DrvTimerReload(uint32_t reload);
  friend void DrvTimerStop(void);

  CanopenNode canopen_node;
  CO_NODE *node;

  uint32_t node_id;

  CanStatus last_status = {};
  uint32_t status_time_ms = 0;
  uint32_t bus_off_time_ms = 0;

  bool waiting_for_bus_recovery = false;
  int recovery_delay_seconds = 60;

  OperationalTrigger *on_operational = {};
  PreOperationalTrigger *on_pre_operational = {};

  std::vector<BaseCanopenEntity *> entities;

  uint16_t heartbeat_interval_ms = 0;

  uint8_t dirty_tpdo_mask = 0;

  ESPPreferenceObject comm_state;
  bool pdo_od_writer_enabled = true;

  void parse_od_writer_frame(CO_IF_FRM *frm);

 public:
  HbConsumerEventTrigger *on_hb_cons_event = {};  // TODO: change visibility
  CanStatus status;
  std::map<uint32_t, std::function<void(void *, uint32_t)>> can_cmd_handlers;

#ifdef USE_CANOPEN_OTA
  CanopenOTAComponent *ota;
  void set_ota(CanopenOTAComponent *ota) { this->ota = ota; }
#endif

  canbus::Canbus *canbus;
  void set_canbus(canbus::Canbus *canbus);

  CanopenComponent(uint32_t node_id);

  void add_trigger(OperationalTrigger *trigger) { on_operational = trigger; }
  void add_trigger(PreOperationalTrigger *trigger) { on_pre_operational = trigger; }
  void add_trigger(HbConsumerEventTrigger *trigger) { on_hb_cons_event = trigger; }
  void set_pre_operational_mode();
  void set_operational_mode();

  void rpdo_map_append(uint8_t idx, uint32_t index, uint8_t sub, uint8_t size);

  void trig_tpdo(int8_t num = -1);

  void setup_csdo(uint8_t num, uint8_t node_id, uint32_t tx_id, uint32_t rx_id);
  void csdo_recv(uint8_t num, uint32_t key, std::function<void(uint32_t, uint32_t)> cb);

  void csdo_send_data(uint8_t num, uint32_t key, uint8_t *data, uint8_t len);
  void csdo_send_u8(uint8_t num, uint32_t key, uint8_t value) { csdo_send_data(num, key, (uint8_t *) (&value), 1); }
  void csdo_send_u16(uint8_t num, uint32_t key, uint16_t value) { csdo_send_data(num, key, (uint8_t *) (&value), 2); }
  void csdo_send_u32(uint8_t num, uint32_t key, uint32_t value) { csdo_send_data(num, key, (uint8_t *) (&value), 4); }
  void csdo_send_float(uint8_t num, uint32_t key, float value) { csdo_send_data(num, key, (uint8_t *) (&value), 4); }

  float get_setup_priority() const override;
  void setup() override;

  bool is_initialized() { return node->Nmt.Mode == CO_PREOP || node->Nmt.Mode == CO_OPERATIONAL; }
  void add_rpdo_dummy(uint8_t idx, uint8_t size);
  void add_rpdo_node(uint8_t idx, uint8_t node_id, uint8_t tpdo);
  void add_rpdo_entity_cmd(uint8_t idx, uint8_t entity_id, uint8_t cmd);

  void enable_pdo_od_writer(bool enable) { pdo_od_writer_enabled = enable; };

#ifdef USE_SENSOR
  void add_entity(sensor::Sensor *sensor, uint32_t entity_id, TPDO tpdo, uint8_t size = 4, float min_val = 0,
                  float max_val = 0) {
    entities.push_back(new SensorEntity(sensor, entity_id, tpdo, size, min_val, max_val));
  }

#endif

#ifdef USE_NUMBER
  void add_entity(esphome::number::Number *number, uint32_t entity_id, TPDO tpdo, uint8_t size = 4, float min_val = 0,
                  float max_val = 0) {
    entities.push_back(new NumberEntity(number, entity_id, tpdo, size, min_val, max_val));
  }
#endif

#ifdef USE_BINARY_SENSOR
  void add_entity(binary_sensor::BinarySensor *sensor, uint32_t entity_id, TPDO tpdo) {
    entities.push_back(new BinarySensorEntity(sensor, entity_id, tpdo));
  }
#endif

#ifdef USE_SWITCH
  void add_entity(esphome::switch_::Switch *switch_, uint32_t entity_id, TPDO tpdo) {
    entities.push_back(new SwitchEntity(switch_, entity_id, tpdo));
  }
#endif

#ifdef USE_LIGHT
  void add_entity(esphome::light::LightState *light, uint32_t entity_id, TPDO tpdo) {
    entities.push_back(new LightStateEntity(light, entity_id, tpdo));
  }
#endif

#ifdef USE_COVER
  void add_entity(esphome::cover::Cover *cover, uint32_t entity_id, TPDO tpdo) {
    entities.push_back(new CoverEntity(cover, entity_id, tpdo));
  }
#endif

#ifdef USE_ALARM_CONTROL_PANEL
  void add_entity(esphome::alarm_control_panel::AlarmControlPanel *alarm, uint32_t entity_id, TPDO tpdo) {
    entities.push_back(new AlarmEntity(alarm, entity_id, tpdo));
  }
#endif

  void od_add_metadata(uint32_t entity_id, uint32_t type, const std::string &name, const std::string &device_class,
                       const std::string &unit, const std::string &state_class);
  void od_add_min_max_metadata(uint32_t entity_id, float min_value, float max_value);

  void od_setup_tpdo(uint32_t index, uint8_t sub_index, uint8_t size, TPDO &tpdo);
  uint32_t od_add_state(uint32_t entity_id, const CO_OBJ_TYPE *type, void *state, uint8_t size, TPDO &tpdo);
  uint32_t od_add_cmd(uint32_t entity_id, std::function<void(void *, uint32_t)> cb, const CO_OBJ_TYPE *type = CO_TCMD8);

  void add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<uint8_t> *trigger);
  void add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<int8_t> *trigger);
  void add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<uint16_t> *trigger);
  void add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<int16_t> *trigger);
  void add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<uint32_t> *trigger);
  void add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<int32_t> *trigger);

  void od_set_string(uint32_t index, uint32_t sub, const char *value);
  void set_heartbeat_interval(uint16_t interval_ms);
  void setup_heartbeat_client(uint8_t subidx, uint8_t node_id, uint16_t timeout_ms);
  int16_t get_heartbeat_events(uint8_t node_id);
  void initiate_recovery();
  void od_set_state(uint32_t key, void *state, uint8_t size);
  void set_entity_state(uint32_t entity_id, uint32_t state, void *data, uint8_t size) {
    od_set_state(ENTITY_STATE_KEY(entity_id, state), data, size);
  }

  bool send_entity_cmd(uint8_t node_id, uint32_t entity_index, uint8_t value, uint8_t cmd = 0) {
    return remote_entity_write_od(node_id, ENTITY_INDEX(entity_index) + 2, cmd + 1, &value, 1);
  }

  bool send_entity_cmd(uint8_t node_id, uint32_t entity_index, bool value, uint8_t cmd = 0) {
    return send_entity_cmd(node_id, entity_index, (uint8_t) (value ? 1 : 0), cmd);
  }

  bool send_entity_cmd(uint8_t node_id, uint32_t entity_index, uint16_t value, uint8_t cmd = 0) {
    return remote_entity_write_od(node_id, ENTITY_INDEX(entity_index) + 2, cmd + 1, &value, 2);
  }

  bool send_entity_cmd(uint8_t node_id, uint32_t entity_index, uint32_t value, uint8_t cmd = 0) {
    return remote_entity_write_od(node_id, ENTITY_INDEX(entity_index) + 2, cmd + 1, &value, 4);
  }

  bool send_entity_cmd(uint8_t node_id, uint32_t entity_index, float value, uint8_t cmd = 0) {
    return remote_entity_write_od(node_id, ENTITY_INDEX(entity_index) + 2, cmd + 1, &value, 4);
  }

  bool remote_entity_write_od(uint8_t node_id, uint32_t index, uint8_t subindex, void *data, uint8_t size);

  void on_frame(uint32_t can_id, bool rtr, std::vector<uint8_t> &data);
  void store_comm_params();
  void reset_comm_params();
  void loop() override;
  bool get_can_status(CanStatus &status_info);
};
extern CanopenComponent *current_canopen;
extern std::vector<CanopenComponent *> all_instances;

}  // namespace canopen
}  // namespace esphome

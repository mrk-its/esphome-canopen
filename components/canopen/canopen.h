#pragma once

#include "esphome.h"
using namespace esphome;

#include "esphome/core/component.h"
#include "esphome/components/canbus/canbus.h"
#include <vector>
#include <driver/twai.h>

#include "node_spec.h"
#include "co_if.h"
#include "co_cmd.h"

const int8_t ENTITY_TYPE_DISABLED = 0;
const int8_t ENTITY_TYPE_SENSOR = 1;
const int8_t ENTITY_TYPE_BINARY_SENSOR = 2;
const int8_t ENTITY_TYPE_SWITCH = 3;
const int8_t ENTITY_TYPE_COVER = 4;
const int8_t ENTITY_TYPE_LIGHT = 5;
const uint8_t ENTITY_TYPE_CAN_STATUS = 254;

const uint8_t ENTITY_INDEX_TYPE = 1;
const uint8_t ENTITY_INDEX_DEVICE_CLASS = 2;
const uint8_t ENTITY_INDEX_NAME = 3;
const uint8_t ENTITY_INDEX_UNIT = 4;

const int32_t PROPERTY_STATE0 = 0;
const int32_t PROPERTY_STATE1= 1;
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
      uint32_t tx_err;
      uint32_t rx_err;
      uint32_t tx_failed;
      uint32_t rx_miss;
      uint32_t arb_lost;
      uint32_t bus_err;
    };
    const uint32_t status_update_interval = 1;

    class OperationalTrigger : public Trigger<> {};
    class PreOperationalTrigger : public Trigger<> {};

    class CanopenComponent : public Component {
      CO_NODE node;
      CanStatus last_status;
      struct timeval status_time;

      void rpdo_map_append(uint8_t idx, uint32_t index, uint8_t sub, uint8_t size);

      OperationalTrigger *on_operational;
      PreOperationalTrigger *on_pre_operational;

      public:
      CanStatus status;
      std::map<uint32_t, std::function< void(void *, uint32_t)>> can_cmd_handlers;

      canbus::Canbus *canbus;
      optional<CO_IF_FRM> recv_frame;

      CanopenComponent(canbus::Canbus *canbus, uint32_t node_id);

      void add_trigger(OperationalTrigger *trigger) {
        on_operational = trigger;
      }
      void add_trigger(PreOperationalTrigger *trigger) {
        on_pre_operational = trigger;
      }
      void set_operational_mode();

      void setup_csdo(uint8_t num, uint8_t node_id, uint32_t tx_id, uint32_t rx_id);
      void csdo_recv(uint8_t num, uint32_t key, std::function<void(uint32_t, uint32_t)> cb);

      void csdo_send_data(uint8_t num, uint32_t key, uint8_t *data, uint8_t len);
      void csdo_send_u8(uint8_t num, uint32_t key, uint8_t value) {
        csdo_send_data(num, key, (uint8_t *)(&value), 1);
      }
      void csdo_send_u16(uint8_t num, uint32_t key, uint16_t value) {
        csdo_send_data(num, key, (uint8_t *)(&value), 2);
      }
      void csdo_send_u32(uint8_t num, uint32_t key, uint32_t value) {
        csdo_send_data(num, key, (uint8_t *)(&value), 4);
      }
      void csdo_send_float(uint8_t num, uint32_t key, float value) {
        csdo_send_data(num, key, (uint8_t *)(&value), 4);
      }

      float get_setup_priority() const override { return setup_priority::HARDWARE - 1.0; }
      void setup();
      bool is_initialized() {
        return node.Nmt.Mode == CO_PREOP || node.Nmt.Mode == CO_OPERATIONAL;
      }
      void add_rpdo_dummy(uint8_t idx, uint8_t size);
      void add_rpdo_node(uint8_t idx, uint8_t node_id, uint8_t tpdo);
      void add_rpdo_entity_cmd(uint8_t idx, uint8_t entity_id, uint8_t cmd);

      // void add_status(uint32_t entity_id, uint32_t update_interval);
      #ifdef LOG_SENSOR
      void add_entity(sensor::Sensor *sensor, uint32_t entity_id, int8_t tpdo, uint8_t size=4, float min_val=0, float max_val=0);
      #endif

      #ifdef LOG_BINARY_SENSOR
      void add_entity(binary_sensor::BinarySensor *sensor, uint32_t entity_id, int8_t tpdo);
      #endif

      #ifdef LOG_SWITCH
      void add_entity(esphome::switch_::Switch* switch_, uint32_t entity_id, int8_t tpdo);
      #endif

      #ifdef USE_LIGHT
      void add_entity(esphome::light::LightState* light, uint32_t entity_id, int8_t tpdo);
      #endif

      #ifdef LOG_COVER
      void add_entity(esphome::cover::Cover* cover, uint32_t entity_id, int8_t tpdo);
      #endif

      void od_add_metadata(
        uint32_t entity_id,
        uint8_t type,
        const std::string &name,
        const std::string &device_class,
        const std::string &unit,
        const std::string &state_class
      );
      uint32_t od_add_state(uint32_t entity_id, const CO_OBJ_TYPE *type, void *state, uint8_t size, int8_t tpdo);
      uint32_t od_add_cmd(uint32_t entity_id, std::function< void(void *, uint32_t)> cb, const CO_OBJ_TYPE *type=CO_TCMD8);

      void od_set_state(uint32_t key, void *state, uint8_t size);
      void on_frame(uint32_t can_id, bool rtr, std::vector<uint8_t> &data);
      void loop() override;
      bool get_can_status(CanStatus &status_info);
    };
  } // namespace canopen
  extern canopen::CanopenComponent *global_canopen;
} // namespace esphome

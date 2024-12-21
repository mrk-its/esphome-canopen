#include <map>
#include "esphome.h"
#include "canopen.h"
#include "fw.h"

extern "C" {
void esp_log(const char *tag, const char *fmt, ...) {
  va_list arglist;
  va_start(arglist, fmt);
  esphome::esp_log_vprintf_(0, tag, __LINE__, fmt, arglist);
  va_end(arglist);
}
}

void CONmtHbConsEvent(CO_NMT *nmt, uint8_t nodeId) {
  auto trigger = ((esphome::canopen::CanopenNode *) nmt->Node)->canopen->on_hb_cons_event;
  if (trigger) {
    trigger->trigger(nodeId);
  };
}

extern struct CO_OBJ_T object_dictionary[APP_OBJ_N];

namespace esphome {
namespace canopen {

std::vector<CanopenComponent *> all_instances;
CanopenComponent *current_canopen = 0;

// use last RPDO for od_writer
const uint32_t OD_WRITER_COB_ID_BASE = CO_COBID_RPDO_DEFAULT(3);

CO_OBJ_STR *od_string(const std::string &str) {
  auto od_str = new CO_OBJ_STR();
  od_str->Offset = 0;
  od_str->Start = (uint8_t *) (new std::string(str))->c_str();
  return od_str;
}

void BaseCanopenEntity::od_set_state(CanopenComponent *canopen, uint32_t key, void *state, uint8_t size) {
  canopen->od_set_state(key, state, size);
  if (!tpdo.is_async) {
    canopen->dirty_tpdo_mask |= (1 << tpdo.number);
  }
}

/* Each software timer needs some memory for managing
 * the lists and states of the timed action events.
 */
static CO_TMR_MEM TmrMem[APP_TMR_N];

/* Each SDO server needs memory for the segmented or
 * block transfer requests.
 */
static uint8_t SdoSrvMem[CO_SSDO_N * CO_SDO_BUF_BYTE];

/* Specify the EMCY error codes with the corresponding
 * error register bit. There is a collection of defines
 * for the predefined emergency codes CO_EMCY_CODE...
 * and for the error register bits CO_EMCY_REG... for
 * readability. You can use plain numbers, too.
 */
static CO_EMCY_TBL AppEmcyTbl[APP_ERR_ID_NUM] = {
    {CO_EMCY_REG_GENERAL, CO_EMCY_CODE_HW_ERR} /* APP_ERR_ID_EEPROM */
};
/* allocate variables for dynamic runtime value in RAM */
static uint8_t Obj1001_00_08 = 0;

/* allocate variables for constant values in FLASH */
const uint32_t Obj1000_00_20 = 0x00000000L;

const uint32_t Obj1014_00_20 = 0x00000080L;

const uint32_t VENDOR_ID = 0xa59a08f5;
const uint32_t PRODUCT_CODE = 0x6bdfa1d9;
const uint32_t REVISION_NUMBER = 0x00000000;
const uint32_t SERIAL_NUMBER = 0x00000000;

const uint32_t Obj1200_01_20 = CO_COBID_SDO_REQUEST();
const uint32_t Obj1200_02_20 = CO_COBID_SDO_RESPONSE();

const struct CO_OBJ_T od_header[] = {
    {CO_KEY(0x1000, 0, CO_OBJ_____R_), CO_TUNSIGNED32, (CO_DATA) (&Obj1000_00_20)},
    {CO_KEY(0x1001, 0, CO_OBJ_____R_), CO_TUNSIGNED8, (CO_DATA) (&Obj1001_00_08)},
    {CO_KEY(0x100a, 0, CO_OBJ_D___R_), CO_TSTRING, (CO_DATA) 0},
    {CO_KEY(0x1010, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA) 2},
    {CO_KEY(0x1010, 1, CO_OBJ_D___RW), CO_TSTORE, (CO_DATA) 0},
    {CO_KEY(0x1010, 2, CO_OBJ_D___RW), CO_TSTORE, (CO_DATA) 0},
    {CO_KEY(0x1011, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA) 2},
    {CO_KEY(0x1011, 1, CO_OBJ_D___RW), CO_TRESET, (CO_DATA) 0},
    {CO_KEY(0x1011, 2, CO_OBJ_D___RW), CO_TRESET, (CO_DATA) 0},

    {CO_KEY(0x1014, 0, CO_OBJ__N__R_), CO_TEMCY_ID, (CO_DATA) (&Obj1014_00_20)},
    {CO_KEY(0x1016, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA) 0x00000000},

    {CO_KEY(0x1018, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA) (4)},
    {CO_KEY(0x1018, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, (CO_DATA) (VENDOR_ID)},
    {CO_KEY(0x1018, 2, CO_OBJ_D___R_), CO_TUNSIGNED32, (CO_DATA) (PRODUCT_CODE)},
    {CO_KEY(0x1018, 3, CO_OBJ_D___R_), CO_TUNSIGNED32, (CO_DATA) (REVISION_NUMBER)},
    {CO_KEY(0x1018, 4, CO_OBJ_D___R_), CO_TUNSIGNED32, (CO_DATA) (SERIAL_NUMBER)},

    {CO_KEY(0x1200, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA) (2)},
    {CO_KEY(0x1200, 1, CO_OBJ__N__R_), CO_TUNSIGNED32, (CO_DATA) (&Obj1200_01_20)},
    {CO_KEY(0x1200, 2, CO_OBJ__N__R_), CO_TUNSIGNED32, (CO_DATA) (&Obj1200_02_20)},
};

CanopenComponent::CanopenComponent(uint32_t node_id) : od(APP_OBJ_N) {
  ESP_LOGI(TAG, "initializing CANopen-stack, node_id: %03x", node_id);
  canopen_node.canopen = this;
  node = &canopen_node.node;

  recv_frames.reserve(3);

  memset(rpdo_buf, 0, sizeof(rpdo_buf));
  this->node_id = node_id;

  for (auto ptr = od_header; ptr < od_header + sizeof(od_header) / sizeof(od_header[0]); ptr++) {
    od.append(ptr->Key, ptr->Type, ptr->Data);
  }

  for (int n = 0; n < CO_RPDO_N; n++) {
    od.append(CO_KEY(0x1400 + n, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA) rpdo_buf[n] + 0);
    od.append(CO_KEY(0x1400 + n, 1, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA) rpdo_buf[n] + 1);
    od.append(CO_KEY(0x1400 + n, 2, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA) rpdo_buf[n] + 5);
    od.append(CO_KEY(0x1400 + n, 3, CO_OBJ_____RW), CO_TUNSIGNED16, (CO_DATA) rpdo_buf[n] + 6);
  };

  for (int n = 0; n < CO_RPDO_N; n++) {
    od.append(CO_KEY(0x1600 + n, 0, CO_OBJ_____RW), CO_TUNSIGNED8, (CO_DATA) rpdo_buf[n] + 8);
    od.append(CO_KEY(0x1600 + n, 1, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA) rpdo_buf[n] + 9);
    od.append(CO_KEY(0x1600 + n, 2, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA) rpdo_buf[n] + 13);
    od.append(CO_KEY(0x1600 + n, 3, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA) rpdo_buf[n] + 17);
    od.append(CO_KEY(0x1600 + n, 4, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA) rpdo_buf[n] + 21);
    od.append(CO_KEY(0x1600 + n, 5, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA) rpdo_buf[n] + 25);
    od.append(CO_KEY(0x1600 + n, 6, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA) rpdo_buf[n] + 29);
    od.append(CO_KEY(0x1600 + n, 7, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA) rpdo_buf[n] + 33);
    od.append(CO_KEY(0x1600 + n, 8, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA) rpdo_buf[n] + 37);
  };

  for (int i = 0; i < 8; i++)
    od.append(CO_KEY(0x1800 + i, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, 0);
  for (int i = 0; i < 8; i++)
    od.append(CO_KEY(0x1A00 + i, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, 0);

  memset(&status, 0, sizeof(status));
  memset(&last_status, 0, sizeof(last_status));

  CO_OBJ_STR *esphome_ver_str = od_string(ESPHOME_VERSION " " + App.get_compilation_time());
  od.add_update(CO_KEY(0x100a, 0, CO_OBJ_____R_), CO_TSTRING, (CO_DATA) esphome_ver_str);
}
void CanopenComponent::set_heartbeat_interval(uint16_t interval_ms) { heartbeat_interval_ms = interval_ms; }

void CanopenComponent::parse_od_writer_frame(CO_IF_FRM *frm) {
  if ((frm->Identifier & ~0x7f) == OD_WRITER_COB_ID_BASE && frm->DLC > 4 && frm->Data[0] == this->node_id) {
    uint32_t key = ((uint32_t *) frm->Data)[0] >> 8;
    uint32_t value = ((uint32_t *) frm->Data)[1];
    uint32_t index = key >> 8;
    uint8_t subindex = (uint8_t) (key & 0xff);

    if (frm->DLC == 5) {
      value = value & 0xff;
    } else if (frm->DLC == 6) {
      value = value & 0xffff;
    } else if (frm->DLC == 7) {
      value = value & 0xffffff;
    }
    ESP_LOGI(TAG, "cmd from: %02x key: %06x value: %08x", frm->Identifier & 0x7f, key, value);
    auto obj = CODictFind(&node->Dict, (key << 8));
    if (!obj) {
      ESP_LOGW(TAG, "Can't find object at %04x %02x", index, subindex);
      return;
    }
    if (COObjWrValue(obj, node, frm->Data + 4, frm->DLC - 4) != CO_ERR_NONE) {
      ESP_LOGW(TAG, "Can't write %d bytes to %04x %02x", frm->DLC - 4, index, subindex);
    }
  }
}

void CanopenComponent::on_frame(uint32_t can_id, bool rtr, std::vector<uint8_t> &data) {
  CO_IF_FRM frame = {can_id, {}, (uint8_t) data.size()};
  memcpy(frame.Data, &data[0], data.size());
  recv_frames.push_back(frame);
  // this assumes single-threded ESPHome callbacks
  current_canopen = this;

  CONodeProcess(node);
  if (pdo_od_writer_enabled)
    parse_od_writer_frame(&frame);

  current_canopen = 0;
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
  };
  lambdaaction = new LambdaAction<std::vector<uint8_t>, uint32_t, bool>(cb);
  automation->add_actions({lambdaaction});
}

void CanopenComponent::od_add_metadata(uint32_t entity_id, uint8_t type, const std::string &name,
                                       const std::string &device_class, const std::string &unit,
                                       const std::string &state_class) {
  uint32_t index = ENTITY_INDEX(entity_id);
  od.add_update(CO_KEY(0x2000, entity_id, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA) type);
  if (name.size())
    od.add_update(CO_KEY(index, ENTITY_INDEX_NAME, CO_OBJ_____R_), CO_TSTRING, (CO_DATA) od_string(name));
  if (device_class.size())
    od.add_update(CO_KEY(index, ENTITY_INDEX_DEVICE_CLASS, CO_OBJ_____R_), CO_TSTRING,
                  (CO_DATA) od_string(device_class));
  if (unit.size())
    od.add_update(CO_KEY(index, ENTITY_INDEX_UNIT, CO_OBJ_____R_), CO_TSTRING, (CO_DATA) od_string(unit));
  if (state_class.size())
    od.add_update(CO_KEY(index, ENTITY_INDEX_STATE_CLASS, CO_OBJ_D___R_), CO_TSTRING, (CO_DATA) od_string(state_class));
}

void CanopenComponent::od_add_sensor_metadata(uint32_t entity_id, float min_value, float max_value) {
  uint32_t index = ENTITY_INDEX(entity_id);
  // temporary pointers to get rid of aliasing warning
  uint32_t *min_value_ptr = (uint32_t *) &min_value;
  uint32_t *max_value_ptr = (uint32_t *) &max_value;
  od.add_update(CO_KEY(index, ENTITY_INDEX_SENSOR_MIN_VALUE, CO_OBJ_D___R_), CO_TUNSIGNED32, (CO_DATA) *min_value_ptr);
  od.add_update(CO_KEY(index, ENTITY_INDEX_SENSOR_MAX_VALUE, CO_OBJ_D___R_), CO_TUNSIGNED32, (CO_DATA) *max_value_ptr);
}

uint32_t CanopenComponent::od_add_state(uint32_t entity_id, const CO_OBJ_TYPE *type, void *state, uint8_t size,
                                        TPDO &tpdo) {
  uint32_t pdo_mask = tpdo.number >= 0 ? (tpdo.is_async ? CO_OBJ___A___ : 0) | CO_OBJ____P__ : 0;
  uint32_t entity_index = ENTITY_INDEX(entity_id);

  uint8_t state_sub_index = 0;
  auto obj = od.find(CO_DEV(entity_index + 1, 0));
  if (obj)
    state_sub_index = obj->Data;
  state_sub_index += 1;

  uint32_t value = 0;
  if (state && size)
    memcpy(&value, state, size);
  od.add_update(CO_KEY(entity_index + 1, state_sub_index, pdo_mask | CO_OBJ_D___R_), type, value);

  if (tpdo.number >= 0) {
    od_setup_tpdo(entity_index + 1, state_sub_index, size, tpdo);
  }
  return CO_KEY(entity_index + 1, state_sub_index, 0);
}

void CanopenComponent::od_setup_tpdo(uint32_t index, uint8_t sub_index, uint8_t size, TPDO &tpdo) {
  od.add_update(CO_KEY(0x1800 + tpdo.number, 1, CO_OBJ_DN__R_), CO_TUNSIGNED32,
                tpdo.number < 4 ? CO_COBID_TPDO_DEFAULT(tpdo.number) : CO_COBID_TPDO_DEFAULT(tpdo.number - 4) + 0x80);
  od.add_update(CO_KEY(0x1800 + tpdo.number, 2, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA) 254);

  uint8_t tpdo_sub_index = 0;
  auto obj = od.find(CO_DEV(0x1a00 + tpdo.number, 0));
  if (obj)
    tpdo_sub_index = obj->Data;
  tpdo_sub_index += 1;
  uint32_t bits = size * 8;
  od.add_update(CO_KEY(0x1a00 + tpdo.number, tpdo_sub_index, CO_OBJ_D___R_), CO_TUNSIGNED32,
                CO_LINK(index, sub_index, bits));
}

void CanopenComponent::od_set_state(uint32_t key, void *state, uint8_t size) {
  auto obj = CODictFind(&node->Dict, key);
  if (!obj)
    return;
  if (!size) {
    size = obj->Type->Size(obj, node, 4);
  }
  COObjWrValue(obj, node, state, size);
}

uint32_t CanopenComponent::od_add_cmd(uint32_t entity_id, std::function<void(void *, uint32_t)> cb,
                                      const CO_OBJ_TYPE *type) {
  uint32_t index = ENTITY_INDEX(entity_id);

  uint8_t max_index = 0;
  auto obj = od.find(CO_DEV(index + 2, 0));
  if (obj)
    max_index = obj->Data;
  max_index += 1;

  od.add_update(CO_KEY(index + 2, max_index, CO_OBJ_D___RW), type, (CO_DATA) 0);
  auto key = CO_KEY(index + 2, max_index, 0);
  can_cmd_handlers[key] = cb;
  return key;
}

void CanopenComponent::rpdo_map_append(uint8_t idx, uint32_t index, uint8_t sub, uint8_t bit_size) {
  auto obj = od.find(CO_DEV(0x1600 + idx, 0));
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
  auto obj = od.find(CO_DEV(0x1400 + idx, 0));
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
  uint32_t index = ENTITY_INDEX(entity_id);
  rpdo_map_append(idx, index + 2, cmd + 1, 8);
}

void CanopenComponent::add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<uint8_t> *trigger) {
  od_add_cmd(entity_id, [=](void *buffer, uint32_t size) { trigger->trigger(*((uint8_t *) buffer)); }, CO_TCMD8);
}

void CanopenComponent::add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<int8_t> *trigger) {
  od_add_cmd(entity_id, [=](void *buffer, uint32_t size) { trigger->trigger(*((int8_t *) buffer)); }, CO_TCMD8);
}

void CanopenComponent::add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<uint16_t> *trigger) {
  od_add_cmd(entity_id, [=](void *buffer, uint32_t size) { trigger->trigger(*((uint16_t *) buffer)); }, CO_TCMD16);
}

void CanopenComponent::add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<int16_t> *trigger) {
  od_add_cmd(entity_id, [=](void *buffer, uint32_t size) { trigger->trigger(*((int16_t *) buffer)); }, CO_TCMD16);
}

void CanopenComponent::add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<uint32_t> *trigger) {
  od_add_cmd(entity_id, [=](void *buffer, uint32_t size) { trigger->trigger(*((uint32_t *) buffer)); }, CO_TCMD32);
}

void CanopenComponent::add_entity_cmd(uint32_t entity_id, int8_t tpdo, Trigger<int32_t> *trigger) {
  od_add_cmd(entity_id, [=](void *buffer, uint32_t size) { trigger->trigger(*((int32_t *) buffer)); }, CO_TCMD32);
}

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
  od.add_update(CO_KEY(0x1280 + num, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, 0);
  od.add_update(CO_KEY(0x1280 + num, 1, CO_OBJ_D___R_), CO_TUNSIGNED32, tx_id);
  od.add_update(CO_KEY(0x1280 + num, 2, CO_OBJ_D___R_), CO_TUNSIGNED32, rx_id);
  od.add_update(CO_KEY(0x1280 + num, 3, CO_OBJ_D___R_), CO_TUNSIGNED8, node_id);
}

void CanopenComponent::od_set_string(uint32_t index, uint32_t sub, const char *value) {
  od.add_update(CO_KEY(index, sub, CO_OBJ_____R_), CO_TSTRING, (CO_DATA) od_string(value));
}

float CanopenComponent::get_setup_priority() const { return setup_priority::PROCESSOR; }

void CanopenComponent::setup() {
  all_instances.push_back(this);
  current_canopen = this;
  ESP_LOGCONFIG(TAG, "Setting up CANopen...");
  ESP_LOGD(TAG, "CO_TPDO_N: %d", CO_TPDO_N);
  ESP_LOGD(TAG, "CO_RPDO_N: %d", CO_RPDO_N);

#ifdef USE_ESP32
  twai_reconfigure_alerts(TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF, NULL);
#endif

  uint32_t hash = fnv1_hash("canopen_comm_state_v2");
  this->comm_state = global_preferences->make_preference<uint8_t[CO_RPDO_N * 41]>(hash, true);
  if (this->comm_state.load(&rpdo_buf)) {
    ESP_LOGI(TAG, "loaded RPDO config from preferences");
  } else {
    ESP_LOGI(TAG, "can't load RPDO config from preferences, using defaults");
  }

  if (heartbeat_interval_ms) {
    od.add_update(CO_KEY(0x1017, 0, CO_OBJ_D___RW), CO_THB_PROD, (CO_DATA) (heartbeat_interval_ms));
  }

  // manufacturer device name
  od_set_string(0x1008, 0, App.get_name().c_str());

  for (uint8_t i = 0; i < 8; i++) {
    od.add_update(CO_KEY(0x1800 + i, 1, CO_OBJ_DN__R_), CO_TUNSIGNED32,
                  i < 4 ? CO_COBID_TPDO_DEFAULT(i) : CO_COBID_TPDO_DEFAULT(i - 4) + 0x80);
    od.add_update(CO_KEY(0x1800 + i, 2, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA) 254);
  }

#ifdef USE_CANOPEN_OTA
  FirmwareObj.domain.Size = 1024 * 1024;

  od.add_update(CO_KEY(0x3000, 1, CO_OBJ_D____W), FW_CTRL, (CO_DATA) 0);
  od.add_update(CO_KEY(0x3000, 2, CO_OBJ_____RW), CO_TUNSIGNED32, (CO_DATA) (&FirmwareObj.size));
  od.add_update(CO_KEY(0x3000, 3, CO_OBJ______W), CO_TDOMAIN, (CO_DATA) (&FirmwareMD5));
  od.add_update(CO_KEY(0x3000, 4, CO_OBJ______W), FW_IMAGE, (CO_DATA) (&FirmwareObj));
#endif

  for (auto it = entities.begin(); it != entities.end(); it++) {
    (*it)->setup(this);
  }

  CO_NODE_SPEC_T NodeSpec = {
      (uint8_t) node_id,    /* default Node-Id                */
      APP_BAUDRATE,         /* default Baudrate               */
      &*od.od.begin(),      /* pointer to object dictionary   */
      APP_OBJ_N,            /* object dictionary max length   */
      AppEmcyTbl,           /* EMCY code & register bit table */
      TmrMem,               /* pointer to timer memory blocks */
      APP_TMR_N,            /* number of timer memory blocks  */
      APP_TICKS_PER_SEC,    /* timer clock frequency in Hz    */
      &CanOpenStack_Driver, /* select drivers for application */
      SdoSrvMem             /* SDO Transfer Buffer Memory     */
  };

  CONodeInit(node, &NodeSpec);
  auto err = CONodeGetErr(node);
  if (err != CO_ERR_NONE) {
    ESP_LOGE(TAG, "canopen init error: %d", err);
  }

  CONodeStart(node);
  set_pre_operational_mode();
}

void CanopenComponent::set_pre_operational_mode() {
  current_canopen = this;
  CONmtSetMode(&node->Nmt, CO_PREOP);
  current_canopen = 0;

  ESP_LOGI(TAG, "node is pre_operational");
  if (on_pre_operational) {
    on_pre_operational->trigger();
  } else {
    set_operational_mode();
  }
}

void CanopenComponent::set_operational_mode() {
  current_canopen = this;
  // update dictionary size
  // as new entries may have been added on pre_operational phase
  node->Dict.Num = od.od.size();
  CONmtSetMode(&node->Nmt, CO_OPERATIONAL);
  current_canopen = 0;

  ESP_LOGD(TAG, "############# Object Dictionary #############");
  uint16_t index = 0;
  for (auto od = node->Dict.Root; index < node->Dict.Num;) {
    uint32_t value = od->Key & CO_OBJ_D_____ ? od->Data : (*(uint32_t *) od->Data);
    if (od->Type && od->Type->Size) {
      auto size = od->Type->Size(od, node, 4);
      if (size == 1)
        value &= 0xff;
      else if (size == 2)
        value &= 0xffff;
    }
    ESP_LOGD(TAG, "OD Index: %02x Key: %08x Data: %08x Type: %p", index, od->Key, value, od->Type);
    index++;
    od++;
  }
  ESP_LOGD(TAG, "#############################################");

  ESP_LOGI(TAG, "node is operational");

  if (on_operational)
    on_operational->trigger();
}

void CanopenComponent::trig_tpdo(int8_t num) {
  current_canopen = this;
  for (auto tpdo = 0; tpdo < CO_TPDO_N; tpdo++) {
    if (node->TPdo[tpdo].ObjNum > 0 && (num < 0 || tpdo == num)) {
      COTPdoTrigPdo(node->TPdo, tpdo);
    }
  }
  current_canopen = 0;
}

bool CanopenComponent::remote_entity_write_od(uint8_t node_id, uint32_t index, uint8_t subindex, void *data,
                                              uint8_t size) {
  if (size > 4 || node_id >= 128) {
    return false;
  }
  if (node_id == this->node_id) {
    uint32_t key = CO_KEY(index, subindex, 0);
    auto obj = CODictFind(&node->Dict, key);
    if (!obj) {
      ESP_LOGW(TAG, "Can't find object at %04x %02x", index, subindex);
      return false;
    }
    auto result = COObjWrValue(obj, node, data, size);
    if (result != CO_ERR_NONE) {
      ESP_LOGW(TAG, "Can't write %d bytes to %04x %02x", size, index, subindex);
    }
  }

  CO_IF_FRM frame = {OD_WRITER_COB_ID_BASE | node_id, {}, (uint8_t) (size + 4)};
  frame.Data[0] = node_id;
  frame.Data[1] = subindex;
  frame.Data[2] = (uint8_t) (index & 0xff);
  frame.Data[3] = (uint8_t) ((index >> 8) & 0xff);
  memcpy(frame.Data + 4, data, size);

  current_canopen = this;
  node->If.Drv->Can->Send(&frame);
  current_canopen = 0;

  // uint8_t buffer[8] = {node_id, subindex, (uint8_t)(index & 0xff), (uint8_t)((index >> 8) & 0xff)};
  // memcpy(buffer + 4, data, size);
  // std::vector<uint8_t> _data(buffer, buffer + 4 + size);
  // canbus->send_data(OD_WRITER_COB_ID_BASE | node_id, false, _data);
  return true;
}

void CanopenComponent::csdo_recv(uint8_t num, uint32_t key, std::function<void(uint32_t, uint32_t)> cb) {
  static auto csdo0 = COCSdoFind(node, 0);
  if (!csdo0) {
    return cb(0, -1);
  }
  static uint32_t buffers[CO_CSDO_N];
  static std::function<void(uint32_t, uint32_t)> callbacks[CO_CSDO_N];
  auto csdo = COCSdoFind(node, num);
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
  auto csdo = COCSdoFind(node, num);
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
#ifdef USE_ESP32
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
#endif
  return false;
}

void CanopenComponent::initiate_recovery() { twai_initiate_recovery(); }

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

int16_t CanopenComponent::get_heartbeat_events(uint8_t node_id) { return CONmtGetHbEvents(&node->Nmt, node_id); }

void CanopenComponent::setup_heartbeat_client(uint8_t subidx, uint8_t node_id, uint16_t timeout_ms) {
  CO_HBCONS *thb_cons = new CO_HBCONS;
  memset(thb_cons, 0, sizeof(CO_HBCONS));
  thb_cons->Time = timeout_ms;
  thb_cons->NodeId = node_id;
  od.add_update(CO_KEY(0x1016, subidx, CO_OBJ_____RW), CO_THB_CONS, (CO_DATA) thb_cons);
}

void CanopenComponent::loop() {
  ESP_LOGVV(TAG, "loop start, node_id: %d", node_id);
  current_canopen = this;
  COTmrService(&node->Tmr);
  COTmrProcess(&node->Tmr);

  while (recv_frames.size() > 0) {
    if (pdo_od_writer_enabled)
      parse_od_writer_frame(&recv_frames[0]);
    CONodeProcess(node);
  }

  current_canopen = 0;

  for (int8_t tpdo_nr = 0; tpdo_nr < 8; tpdo_nr++) {
    if (dirty_tpdo_mask & (1 << tpdo_nr)) {
      ESP_LOGD(TAG, "sending dirty tpdo #%d", tpdo_nr);
      trig_tpdo(tpdo_nr);
    }
  }
  dirty_tpdo_mask = 0;

  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);

  if (abs(tv_now.tv_sec - status_time.tv_sec) >= status_update_interval) {
    if (get_can_status(status)) {
      if (status.tx_err > last_status.tx_err || status.rx_err > last_status.rx_err ||
          status.tx_failed > last_status.tx_failed || status.rx_miss > last_status.rx_miss ||
          status.arb_lost > last_status.arb_lost || status.bus_err > last_status.bus_err) {
        ESP_LOGW(TAG, "tx_err: %d rx_err: %d tx_failed: %d rx_miss: %d arb_lost: %d bus_err: %d", status.tx_err,
                 status.rx_err, status.tx_failed, status.rx_miss, status.arb_lost, status.bus_err);
      }
      last_status = status;
    }
    status_time = tv_now;
  }

#ifdef USE_ESP32

  uint32_t alerts;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
      ESP_LOGI(TAG, "Surpassed Error Warning Limit");
    }
    if (alerts & TWAI_ALERT_ERR_PASS) {
      ESP_LOGI(TAG, "Entered Error Passive state");
    }
    if (alerts & TWAI_ALERT_BUS_OFF) {
      ESP_LOGI(TAG, "Bus Off state");
      // Prepare to initiate bus recovery, reconfigure alerts to detect bus recovery completion
      twai_reconfigure_alerts(TWAI_ALERT_BUS_RECOVERED, NULL);
      waiting_for_bus_recovery = true;
      gettimeofday(&bus_off_time, NULL);
      ESP_LOGI(TAG, "Initiate bus recovery in %d seconds", recovery_delay_seconds);
    }
    if (alerts & TWAI_ALERT_BUS_RECOVERED) {
      // Bus recovery was successful, exit control task to uninstall driver
      ESP_LOGI(TAG, "Bus Recovered");
      if (twai_start() == ESP_OK) {
        ESP_LOGI(TAG, "Bus Running");
        twai_reconfigure_alerts(TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF, NULL);
      } else {
        ESP_LOGE(TAG, "Can't start the Bus");
      }
    }
  }

  if (waiting_for_bus_recovery) {
    auto t = abs(tv_now.tv_sec - bus_off_time.tv_sec);
    if (t >= recovery_delay_seconds) {
      twai_initiate_recovery();  // Needs 128 occurrences of bus free signal
      ESP_LOGI(TAG, "Initiate bus recovery");
      waiting_for_bus_recovery = false;
    }
  }
#endif
  ESP_LOGVV(TAG, "loop end, node_id: %d", node_id);
}
}  // namespace canopen
}  // namespace esphome

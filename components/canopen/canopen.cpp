// CANopenComponent: setup (building the OD and initialising CANopenNode), processing, our own NMT state. The other
// features are in their own files, see canopen.h.
#include "canopen.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::canopen {

ESPHOME_LOG_TAG(TAG, "canopen");

const LogString *nmt_state_to_string(NMTState state) {
  switch (state) {
    case NMTState::NMT_STATE_BOOTUP:
      return LOG_STR("bootup");
    case NMTState::NMT_STATE_STOPPED:
      return LOG_STR("stopped");
    case NMTState::NMT_STATE_OPERATIONAL:
      return LOG_STR("operational");
    case NMTState::NMT_STATE_PRE_OPERATIONAL:
      return LOG_STR("pre-operational");
    default:
      return LOG_STR("unknown");
  }
}

NMTState to_nmt_state(CO_NMT_internalState_t state) {
  switch (state) {
    case CO_NMT_INITIALIZING:
      return NMTState::NMT_STATE_BOOTUP;
    case CO_NMT_STOPPED:
      return NMTState::NMT_STATE_STOPPED;
    case CO_NMT_OPERATIONAL:
      return NMTState::NMT_STATE_OPERATIONAL;
    case CO_NMT_PRE_OPERATIONAL:
      return NMTState::NMT_STATE_PRE_OPERATIONAL;
    default:
      return NMTState::NMT_STATE_UNKNOWN;
  }
}

void CANopenComponent::setup() {
  auto &od = this->od_;
  od.add_var(0x1001, ODA_SDO_R, &this->err_reg_, sizeof(this->err_reg_));
  od.add_var(0x1014, ODA_SDO_RW, &this->cob_emcy_, sizeof(this->cob_emcy_));
  od.add_var(0x1017, ODA_SDO_RW, &this->hb_producer_ms_, sizeof(this->hb_producer_ms_));
  this->add_device_info_od_();
  this->add_node_monitor_od_();
#ifdef USE_CANOPEN_SDO_CLIENT
  this->add_sdo_client_od_();
#endif
#ifdef USE_CANOPEN_PDO
  this->add_pdo_storage_od_();
  this->add_pdo_od_();
#endif
  if (!od.finalize()) {
    this->mark_failed(LOG_STR("invalid object dictionary"));
    return;
  }

  auto &cfg = this->config_;
  cfg.CNT_NMT = 1;
  cfg.ENTRY_H1017 = od.find(0x1017);
  cfg.CNT_EM = 1;
  cfg.ENTRY_H1001 = od.find(0x1001);
  cfg.ENTRY_H1014 = od.find(0x1014);
  cfg.CNT_ARR_1003 = 8;  // EM fifo size, needs >= 1
  this->config_device_info_(cfg);
  this->config_node_monitor_(cfg);
#ifdef USE_CANOPEN_SDO_CLIENT
  this->config_sdo_client_(cfg);
#endif
#ifdef USE_CANOPEN_PDO
  this->config_pdo_(cfg);
#endif

  uint32_t heap = 0;
  this->co_ = CO_new(&cfg, &heap);
  if (this->co_ == nullptr) {
    this->mark_failed(LOG_STR("CO_new failed"));
    return;
  }
  CO_ReturnError_t err = CO_CANinit(this->co_, this, 0);
  if (err != CO_ERROR_NO) {
    ESP_LOGE(TAG, "CO_CANinit failed: %d", err);
    this->mark_failed();
    return;
  }
#ifdef USE_CANOPEN_PDO
  if (!this->init_pdo_storage_()) {
    this->mark_failed();
    return;
  }
#endif
  uint32_t err_info = 0;
  uint16_t nmt_control = this->start_operational_ ? CO_NMT_STARTUP_TO_OPERATIONAL : 0;
  err = CO_CANopenInit(this->co_, nullptr, nullptr, od.od(), nullptr, nmt_control, 500 /* first HB, ms */, 1000, 0,
                       false, this->node_id_, &err_info);
  if (err != CO_ERROR_NO) {
    ESP_LOGE(TAG, "CO_CANopenInit failed: %d (OD 0x%04" PRIx32 ")", err, err_info);
    this->mark_failed();
    return;
  }
#ifdef USE_CANOPEN_PDO
  if (!this->init_pdo_()) {
    this->mark_failed();
    return;
  }
#endif
  this->init_node_monitor_();
  this->init_frame_listeners_();

  CO_CANsetNormalMode(this->co_->CANmodule);
  this->last_us_ = micros();
  ESP_LOGI(TAG, "CANopenNode initialized, node 0x%02x, heap %" PRIu32 " B", this->node_id_, heap);
}

void CANopenComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "CANopen (CANopenNode):");
  ESP_LOGCONFIG(TAG, "  Node ID: 0x%02x", this->node_id_);
  ESP_LOGCONFIG(TAG, "  Heartbeat interval: %u ms", this->hb_producer_ms_);
#ifdef CANOPEN_SDO_BLOCK_SIZE
  ESP_LOGCONFIG(TAG, "  SDO server: block transfer, download block size %u", CANOPEN_SDO_BLOCK_SIZE);
#endif
  ESP_LOGCONFIG(TAG, "  NMT startup: %s",
                this->start_operational_ ? LOG_STR_LITERAL("operational") : LOG_STR_LITERAL("pre-operational"));
  this->dump_device_info_config_();
#ifdef USE_CANOPEN_SDO_CLIENT
  ESP_LOGCONFIG(TAG, "  SDO client: response timeout %u ms", this->response_timeout_ms_);
#endif
  this->dump_node_monitor_config_();
}

void CANopenComponent::loop() { this->process_(); }

void CANopenComponent::process_() {
  if (this->co_ == nullptr || this->is_failed())
    return;
  uint32_t now = micros();
  uint32_t dt = now - this->last_us_;
  this->last_us_ = now;
  CO_NMT_reset_cmd_t reset = CO_process(this->co_, false, dt, nullptr);
  if (reset == CO_RESET_APP) {
    ESP_LOGW(TAG, "NMT reset node");
    App.safe_reboot();
  } else if (reset == CO_RESET_COMM) {
    ESP_LOGW(TAG, "NMT reset communication (not implemented)");
  }
#ifdef USE_CANOPEN_PDO
  this->process_pdo_(dt);
#endif
  // CANopenNode's state change callback has no context pointer, so the state is compared here
  NMTState state = to_nmt_state(CO_NMT_getInternalState(this->co_->NMT));
  if (state != this->nmt_state_) {
    this->nmt_state_ = state;
    ESP_LOGI(TAG, "NMT state: %s", LOG_STR_ARG(nmt_state_to_string(state)));
    this->nmt_state_callback_.call(state);
  }
#ifdef USE_CANOPEN_SDO_CLIENT
  this->process_sdo_(dt);
#endif
}

bool CANopenComponent::set_nmt_state(NMTState state) {
  if (this->co_ == nullptr || this->is_failed())
    return false;
  CO_NMT_command_t command;
  switch (state) {
    case NMTState::NMT_STATE_OPERATIONAL:
      command = CO_NMT_ENTER_OPERATIONAL;
      break;
    case NMTState::NMT_STATE_PRE_OPERATIONAL:
      command = CO_NMT_ENTER_PRE_OPERATIONAL;
      break;
    case NMTState::NMT_STATE_STOPPED:
      command = CO_NMT_ENTER_STOPPED;
      break;
    default:
      return false;
  }
  // applied by the next CO_process()
  CO_NMT_sendInternalCommand(this->co_->NMT, command);
  return true;
}

}  // namespace esphome::canopen

#include "canopen.h"

#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace canopen {

static const char *const TAG = "canopen";

static const char *nmt_state_str(CO_NMT_internalState_t state) {
  switch (state) {
    case CO_NMT_INITIALIZING:
      return "initializing";
    case CO_NMT_PRE_OPERATIONAL:
      return "pre-operational";
    case CO_NMT_OPERATIONAL:
      return "operational";
    case CO_NMT_STOPPED:
      return "stopped";
    default:
      return "unknown";
  }
}

const char *nmt_state_to_string(NMTState state) {
  switch (state) {
    case NMTState::BOOTUP:
      return "bootup";
    case NMTState::STOPPED:
      return "stopped";
    case NMTState::OPERATIONAL:
      return "operational";
    case NMTState::PRE_OPERATIONAL:
      return "pre-operational";
    default:
      return "unknown";
  }
}

void CANopenComponent::set_canbus(canbus::Canbus *canbus) {
  this->canbus_ = canbus;
  canbus->add_callback([this](uint32_t can_id, bool extended_id, bool rtr, const std::vector<uint8_t> &data) {
    this->on_frame_(can_id, extended_id, rtr, data);
  });
}

void CANopenComponent::setup() {
  auto &od = this->od_;
  od.add_var(0x1000, ODA_SDO_R, &this->dev_type_, sizeof(this->dev_type_));
  od.add_var(0x1001, ODA_SDO_R, &this->err_reg_, sizeof(this->err_reg_));
  od.add_var(0x1014, ODA_SDO_RW, &this->cob_emcy_, sizeof(this->cob_emcy_));
  od.add_var(0x1017, ODA_SDO_RW, &this->hb_producer_ms_, sizeof(this->hb_producer_ms_));
  uint8_t n_cons = this->hb_cons_.size();
  if (n_cons) {
    this->hb_cons_count_ = n_cons;
    od.add_array(0x1016, ODA_SDO_RW, &this->hb_cons_count_, this->hb_cons_.data(), sizeof(uint32_t), n_cons);
  }
  od.finalize();

  auto &cfg = this->config_;
  cfg.CNT_NMT = 1;
  cfg.ENTRY_H1017 = od.find(0x1017);
  cfg.CNT_EM = 1;
  cfg.ENTRY_H1001 = od.find(0x1001);
  cfg.ENTRY_H1014 = od.find(0x1014);
  cfg.CNT_ARR_1003 = 8;  // EM fifo size, needs >= 1
  cfg.CNT_HB_CONS = n_cons ? 1 : 0;
  cfg.CNT_ARR_1016 = n_cons;
  cfg.ENTRY_H1016 = od.find(0x1016);

  uint32_t heap = 0;
  this->co_ = CO_new(&cfg, &heap);
  if (this->co_ == nullptr) {
    ESP_LOGE(TAG, "CO_new failed");
    this->mark_failed();
    return;
  }
  CO_ReturnError_t err = CO_CANinit(this->co_, this, 0);
  if (err != CO_ERROR_NO) {
    ESP_LOGE(TAG, "CO_CANinit failed: %d", err);
    this->mark_failed();
    return;
  }
  uint32_t err_info = 0;
  err = CO_CANopenInit(this->co_, nullptr, nullptr, od.od(), nullptr, CO_NMT_STARTUP_TO_OPERATIONAL,
                       500 /* first HB, ms */, 1000, 0, false, this->node_id_, &err_info);
  if (err != CO_ERROR_NO) {
    ESP_LOGE(TAG, "CO_CANopenInit failed: %d (OD 0x%04" PRIx32 ")", err, err_info);
    this->mark_failed();
    return;
  }

  CO_NMT_initCallbackChanged(this->co_->NMT, on_nmt_changed_);
  for (uint8_t i = 0; i < n_cons; i++) {
    CO_HBconsumer_t *hb = this->co_->HBcons;
    CO_HBconsumer_initCallbackHeartbeatStarted(hb, i, this, on_hb_started_);
    CO_HBconsumer_initCallbackNmtChanged(hb, i, this, on_hb_nmt_changed_);
    CO_HBconsumer_initCallbackTimeout(hb, i, this, on_hb_timeout_);
    CO_HBconsumer_initCallbackRemoteReset(hb, i, this, on_hb_remote_reset_);
  }

  CO_CANsetNormalMode(this->co_->CANmodule);
  this->last_us_ = micros();
  ESP_LOGI(TAG, "CANopenNode initialized, node 0x%02x, heap %" PRIu32 " B", this->node_id_, heap);
}

void CANopenComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "CANopen (CANopenNode):");
  ESP_LOGCONFIG(TAG, "  Node ID: 0x%02x", this->node_id_);
  ESP_LOGCONFIG(TAG, "  Heartbeat interval: %u ms", this->hb_producer_ms_);
  for (uint32_t c : this->hb_cons_)
    ESP_LOGCONFIG(TAG, "  Heartbeat client: node 0x%02x, timeout %u ms", (unsigned) (c >> 16), (unsigned) (c & 0xFFFF));
}

void CANopenComponent::on_frame_(uint32_t can_id, bool extended_id, bool rtr, const std::vector<uint8_t> &data) {
  if (extended_id || this->co_ == nullptr || this->is_failed())
    return;
  CO_CANmodule_t *mod = this->co_->CANmodule;
  CO_CANrxMsg_t msg{};
  msg.ident = (can_id & 0x7FF) | (rtr ? 0x800 : 0);
  msg.DLC = data.size() > 8 ? 8 : data.size();
  memcpy(msg.data, data.data(), msg.DLC);
  for (uint16_t i = 0; i < mod->rxSize; i++) {
    CO_CANrx_t *rx = &mod->rxArray[i];
    if (rx->CANrx_callback != nullptr && ((msg.ident ^ rx->ident) & rx->mask) == 0)
      rx->CANrx_callback(rx->object, &msg);
  }
  if (!rtr) {
    uint8_t node;
    NMTState state;
    Emergency emcy;
    if (decode_heartbeat(msg.ident, msg.data, msg.DLC, node, state)) {
      this->heartbeat_callback_.call(node, state);
    } else if (decode_emcy(msg.ident, msg.data, msg.DLC, node, emcy)) {
      ESP_LOGW(TAG, "node 0x%02x EMCY: error code 0x%04x, register 0x%02x", node, emcy.error_code,
               emcy.error_register);
      this->emcy_callback_.call(node, emcy);
    }
  }
  this->process_();
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
}

void CANopenComponent::on_hb_started_(uint8_t node_id, uint8_t idx, void *object) {
  ESP_LOGI(TAG, "node 0x%02x heartbeat started", node_id);
}

void CANopenComponent::on_hb_nmt_changed_(uint8_t node_id, uint8_t idx, CO_NMT_internalState_t state, void *object) {
  ESP_LOGI(TAG, "node 0x%02x NMT state: %s", node_id, nmt_state_str(state));
}

void CANopenComponent::on_hb_timeout_(uint8_t node_id, uint8_t idx, void *object) {
  ESP_LOGW(TAG, "node 0x%02x heartbeat timeout", node_id);
  auto *self = static_cast<CANopenComponent *>(object);
  if (self->on_hb_cons_event_ != nullptr)
    self->on_hb_cons_event_->trigger(node_id);
}

void CANopenComponent::on_hb_remote_reset_(uint8_t node_id, uint8_t idx, void *object) {
  ESP_LOGI(TAG, "node 0x%02x reset (boot-up)", node_id);
}

void CANopenComponent::on_nmt_changed_(CO_NMT_internalState_t state) {
  ESP_LOGI(TAG, "NMT state: %s", nmt_state_str(state));
}

}  // namespace canopen
}  // namespace esphome

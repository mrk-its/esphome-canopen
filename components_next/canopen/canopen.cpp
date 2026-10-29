#include "canopen.h"

#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace canopen {

ESPHOME_LOG_TAG(TAG, "canopen");

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

const char *node_event_to_string(NodeEvent event) {
  switch (event) {
    case NodeEvent::ONLINE:
      return "online";
    case NodeEvent::OFFLINE:
      return "offline";
    case NodeEvent::NMT_CHANGED:
      return "NMT changed";
    case NodeEvent::RESET:
      return "reset";
    default:
      return "unknown";
  }
}

static NMTState to_nmt_state(CO_NMT_internalState_t state) {
  switch (state) {
    case CO_NMT_INITIALIZING:
      return NMTState::BOOTUP;
    case CO_NMT_STOPPED:
      return NMTState::STOPPED;
    case CO_NMT_OPERATIONAL:
      return NMTState::OPERATIONAL;
    case CO_NMT_PRE_OPERATIONAL:
      return NMTState::PRE_OPERATIONAL;
    default:
      return NMTState::UNKNOWN;
  }
}

void CANopenComponent::monitor_node(uint8_t node_id, uint16_t timeout_ms) {
  if (this->co_ != nullptr) {
    ESP_LOGE(TAG, "monitor_node(0x%02x) after setup, ignored", node_id);
    return;
  }
  // CANopenNode rejects a node id that is in 0x1016 twice
  for (uint32_t &c : this->hb_cons_) {
    if ((c >> 16) == node_id) {
      if (timeout_ms < (c & 0xFFFF))
        c = ((uint32_t) node_id << 16) | timeout_ms;
      return;
    }
  }
  this->hb_cons_.push_back(((uint32_t) node_id << 16) | timeout_ms);
}

NMTState CANopenComponent::get_node_state(uint8_t node_id) {
  if (this->co_ == nullptr || this->hb_cons_.empty())
    return NMTState::UNKNOWN;
  int8_t idx = CO_HBconsumer_getIdxByNodeId(this->co_->HBcons, node_id);
  CO_NMT_internalState_t state;
  if (idx < 0 || CO_HBconsumer_getNmtState(this->co_->HBcons, idx, &state) != 0)
    return NMTState::UNKNOWN;
  return to_nmt_state(state);
}

bool CANopenComponent::is_node_online(uint8_t node_id) {
  if (this->co_ == nullptr || this->hb_cons_.empty())
    return false;
  int8_t idx = CO_HBconsumer_getIdxByNodeId(this->co_->HBcons, node_id);
  return idx >= 0 && CO_HBconsumer_getState(this->co_->HBcons, idx) == CO_HBconsumer_ACTIVE;
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

  for (const auto &l : this->frame_listeners_)
    this->check_frame_listener_(l.can_id, l.mask);

  CO_CANsetNormalMode(this->co_->CANmodule);
  this->last_us_ = micros();
  ESP_LOGI(TAG, "CANopenNode initialized, node 0x%02x, heap %" PRIu32 " B", this->node_id_, heap);
}

void CANopenComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "CANopen (CANopenNode):");
  ESP_LOGCONFIG(TAG, "  Node ID: 0x%02x", this->node_id_);
  ESP_LOGCONFIG(TAG, "  Heartbeat interval: %u ms", this->hb_producer_ms_);
  for (uint32_t c : this->hb_cons_) {
    ESP_LOGCONFIG(TAG, "  Heartbeat client: node 0x%02x, timeout %u ms", (unsigned) (c >> 16), (unsigned) (c & 0xFFFF));
  }
}

void CANopenComponent::add_frame_listener(uint32_t can_id, uint32_t mask, FrameCallback callback, void *context) {
  this->frame_listeners_.push_back(FrameListener{can_id & 0x7FF, mask & 0x7FF, callback, context});
  if (this->co_ != nullptr)
    this->check_frame_listener_(can_id & 0x7FF, mask & 0x7FF);
}

// A listener on an ID CANopenNode receives itself (NMT, heartbeats it consumes, ...) is almost always a mistake.
void CANopenComponent::check_frame_listener_(uint32_t can_id, uint32_t mask) {
  CO_CANmodule_t *mod = this->co_->CANmodule;
  for (uint16_t i = 0; i < mod->rxSize; i++) {
    const CO_CANrx_t *rx = &mod->rxArray[i];
    if (rx->CANrx_callback == nullptr || (rx->ident & 0x800))
      continue;
    if (((can_id ^ rx->ident) & mask & rx->mask & 0x7FF) == 0) {
      ESP_LOGW(TAG, "frame listener 0x%03" PRIx32 "/0x%03" PRIx32 " overlaps CAN ID 0x%03x used by CANopenNode", can_id,
               mask, rx->ident & 0x7FF);
    }
  }
}

bool CANopenComponent::send_frame(uint32_t can_id, const uint8_t *data, uint8_t len) {
  if (this->canbus_ == nullptr || len > 8)
    return false;
  return this->canbus_->send_data(can_id & 0x7FF, false, false, data, len) == canbus::ERROR_OK;
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
    for (const auto &l : this->frame_listeners_) {
      if (((msg.ident ^ l.can_id) & l.mask) == 0)
        l.callback(l.context, msg.ident, msg.data, msg.DLC);
    }
    uint8_t node;
    NMTState state;
    Emergency emcy;
    if (decode_heartbeat(msg.ident, msg.data, msg.DLC, node, state)) {
      this->heartbeat_callback_.call(node, state);
    } else if (decode_emcy(msg.ident, msg.data, msg.DLC, node, emcy)) {
      ESP_LOGW(TAG, "node 0x%02x EMCY: error code 0x%04x, register 0x%02x", node, emcy.error_code, emcy.error_register);
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

void CANopenComponent::node_event_(uint8_t node_id, NodeEvent event, NMTState state) {
  if (event == NodeEvent::OFFLINE) {
    ESP_LOGW(TAG, "node 0x%02x offline (heartbeat timeout)", node_id);
  } else {
    ESP_LOGI(TAG, "node 0x%02x %s, NMT state: %s", node_id, node_event_to_string(event), nmt_state_to_string(state));
  }
  this->node_event_callback_.call(node_id, event, state);
}

// CANopenNode heartbeat consumer callbacks (called from CO_process())
void CANopenComponent::on_hb_started_(uint8_t node_id, uint8_t idx, void *object) {
  auto *self = static_cast<CANopenComponent *>(object);
  // the heartbeat's state is already stored, but CO_HBconsumer_getNmtState() only returns it once HBstate is ACTIVE,
  // which CANopenNode sets after this callback
  CO_NMT_internalState_t state = self->co_->HBcons->monitoredNodes[idx].NMTstate;
  self->node_event_(node_id, NodeEvent::ONLINE, to_nmt_state(state));
}

void CANopenComponent::on_hb_nmt_changed_(uint8_t node_id, uint8_t idx, CO_NMT_internalState_t state, void *object) {
  static_cast<CANopenComponent *>(object)->node_event_(node_id, NodeEvent::NMT_CHANGED, to_nmt_state(state));
}

void CANopenComponent::on_hb_timeout_(uint8_t node_id, uint8_t idx, void *object) {
  static_cast<CANopenComponent *>(object)->node_event_(node_id, NodeEvent::OFFLINE, NMTState::UNKNOWN);
}

void CANopenComponent::on_hb_remote_reset_(uint8_t node_id, uint8_t idx, void *object) {
  static_cast<CANopenComponent *>(object)->node_event_(node_id, NodeEvent::RESET, NMTState::BOOTUP);
}

void CANopenComponent::on_nmt_changed_(CO_NMT_internalState_t state) {
  ESP_LOGI(TAG, "NMT state: %s", nmt_state_str(state));
}

}  // namespace canopen
}  // namespace esphome

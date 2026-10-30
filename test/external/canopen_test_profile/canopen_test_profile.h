#pragma once

#include "esphome/components/canopen/canopen_node.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"

namespace esphome {
namespace canopen_test_profile {

using canopen::NMTState;
using canopen::SDOReply;
using canopen::SDOResult;

// Logs every hook of its node and, when the node comes online, checks the SDO forwarders: a read of 0x1018:1 with
// the profile as default context, then a read cancelled right away with cancel_sdo() (no callback expected).
class TestProfile : public Component, public canopen::CANopenNode {
 public:
  TestProfile(canopen::CANopenComponent *parent, uint8_t node_id) : CANopenNode(parent, node_id) {}

  void setup() override {
    this->set_interval(10000, [this]() {
      ESP_LOGI(TAG, "node 0x%02x: online %s, state %s, operational %s", this->get_node_id(),
               YESNO(this->is_online()), canopen::nmt_state_to_string(this->get_nmt_state()),
               YESNO(this->is_operational()));
    });
  }
  void dump_config() override { ESP_LOGCONFIG(TAG, "CANopen test profile for node 0x%02x", this->get_node_id()); }

 protected:
  static constexpr const char *TAG = "canopen_test_profile";

  void on_node_online() override {
    ESP_LOGI(TAG, "hook: online (node 0x%02x)", this->get_node_id());
    this->read_sdo<uint32_t>(0x1018, 1, +[](void *context, const SDOReply &reply) {
      auto *self = CANopenNode::from_context<TestProfile>(context);
      ESP_LOGI(TAG, "forwarded read 0x1018:1: %s, value 0x%08x, context is the profile: %s",
               canopen::sdo_result_to_string(reply.result), (unsigned) reply.value, YESNO(self->is_self_(context)));
      // cancelled right away: must never call back
      bool queued = self->read_sdo<uint32_t>(0x1018, 2, +[](void *, const SDOReply &) {
        ESP_LOGE(TAG, "FAIL: callback of a cancelled request");
      });
      self->cancel_sdo();
      ESP_LOGI(TAG, "read 0x1018:2 %s, cancelled (expect no callback)", queued ? "queued" : "rejected");
    });
  }
  void on_node_offline() override { ESP_LOGW(TAG, "hook: offline (node 0x%02x)", this->get_node_id()); }
  void on_node_nmt_changed(NMTState state) override {
    ESP_LOGI(TAG, "hook: NMT changed (node 0x%02x): %s", this->get_node_id(), canopen::nmt_state_to_string(state));
  }
  void on_node_reset() override { ESP_LOGI(TAG, "hook: reset (node 0x%02x)", this->get_node_id()); }
  void on_node_emcy(const canopen::Emergency &emergency) override {
    ESP_LOGW(TAG, "hook: EMCY (node 0x%02x) code 0x%04x", this->get_node_id(), emergency.error_code);
  }

  bool is_self_(void *context) const { return context == static_cast<const CANopenNode *>(this); }
};

}  // namespace canopen_test_profile
}  // namespace esphome

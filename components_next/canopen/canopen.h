#pragma once

#include <vector>

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/canbus/canbus.h"

#include "od.h"
#include "types.h"

namespace esphome {
namespace canopen {

class CANopenComponent : public Component {
 public:
  explicit CANopenComponent(uint8_t node_id) : node_id_(node_id) {}

  void set_canbus(canbus::Canbus *canbus);
  void set_heartbeat_interval(uint32_t ms) { this->hb_producer_ms_ = ms; }

  // Heartbeat monitoring of another node (CANopenNode heartbeat consumer, OD 0x1016). Before setup() only (called
  // from codegen). Registering a node again keeps the shortest timeout.
  void monitor_node(uint8_t node_id, uint16_t timeout_ms);
  // events of all monitored nodes: (node_id, event, state)
  template<typename F> void add_on_node_event_callback(F &&callback) {
    this->node_event_callback_.add(std::forward<F>(callback));
  }
  // events of one monitored node: (event, state)
  template<typename F> void add_on_node_event(uint8_t node_id, F &&callback) {
    this->node_event_callback_.add([node_id, callback](uint8_t node, NodeEvent event, NMTState state) {
      if (node == node_id)
        callback(event, state);
    });
  }
  // state of a monitored node from its last heartbeat; UNKNOWN / false for nodes that aren't monitored
  NMTState get_node_state(uint8_t node_id);
  bool is_node_online(uint8_t node_id);

  // every heartbeat / EMCY frame on the bus, from any node (our own frames aren't received)
  template<typename F> void add_on_heartbeat_callback(F &&callback) {
    this->heartbeat_callback_.add(std::forward<F>(callback));
  }
  template<typename F> void add_on_emcy_callback(F &&callback) { this->emcy_callback_.add(std::forward<F>(callback)); }

  // Raw frames the OD doesn't describe (e.g. other devices' PDOs). The callback gets every standard-ID data frame
  // with (can_id ^ id) & mask == 0, after CANopenNode has seen it. Can be called before or after setup().
  void add_frame_listener(uint32_t can_id, uint32_t mask, FrameCallback callback, void *context);
  // Sends a standard-ID data frame now. Returns false if the canbus platform refuses it (e.g. TX buffers full); it
  // isn't queued or retried, unlike CANopenNode's own frames.
  bool send_frame(uint32_t can_id, const uint8_t *data, uint8_t len);

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  // used by the CANopenNode driver (driver_can.cpp)
  canbus::Canbus *get_canbus() { return this->canbus_; }

 protected:
  void on_frame_(uint32_t can_id, bool extended_id, bool rtr, const std::vector<uint8_t> &data);
  void process_();
  void check_frame_listener_(uint32_t can_id, uint32_t mask);
  void node_event_(uint8_t node_id, NodeEvent event, NMTState state);

  static void on_hb_started_(uint8_t node_id, uint8_t idx, void *object);
  static void on_hb_nmt_changed_(uint8_t node_id, uint8_t idx, CO_NMT_internalState_t state, void *object);
  static void on_hb_timeout_(uint8_t node_id, uint8_t idx, void *object);
  static void on_hb_remote_reset_(uint8_t node_id, uint8_t idx, void *object);
  static void on_nmt_changed_(CO_NMT_internalState_t state);

  canbus::Canbus *canbus_{nullptr};
  uint8_t node_id_;
  CO_t *co_{nullptr};
  CO_config_t config_{};  // must outlive co_
  ObjectDictionary od_;

  // OD storage
  uint32_t dev_type_{0};           // 0x1000
  uint8_t err_reg_{0};             // 0x1001
  uint32_t cob_emcy_{0x80};        // 0x1014, CANopenNode adds the node id
  uint16_t hb_producer_ms_{5000};  // 0x1017
  uint8_t hb_cons_count_{0};       // 0x1016:0
  std::vector<uint32_t> hb_cons_;  // 0x1016:1.., (node_id << 16) | timeout_ms

  uint32_t last_us_{0};
  LazyCallbackManager<void(uint8_t, NodeEvent, NMTState)> node_event_callback_;
  struct FrameListener {
    uint32_t can_id;
    uint32_t mask;
    FrameCallback callback;
    void *context;
  };
  std::vector<FrameListener> frame_listeners_;
  LazyCallbackManager<void(uint8_t, NMTState)> heartbeat_callback_;
  LazyCallbackManager<void(uint8_t, const Emergency &)> emcy_callback_;
};

}  // namespace canopen
}  // namespace esphome

#pragma once

#include <array>
#include <vector>

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
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
  void set_response_timeout(uint16_t ms) { this->response_timeout_ms_ = ms; }

  // Heartbeat monitoring of another node (CANopenNode heartbeat consumer, OD 0x1016). Before setup() only (called
  // from codegen). Registering a node again keeps the shortest timeout.
  void monitor_node(uint8_t node_id, uint16_t timeout_ms);
  // events of all monitored nodes: (node_id, event, state)
  template<typename F> void add_on_node_event_callback(F &&callback) {
    this->node_event_callback_.add(std::forward<F>(callback));
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

#ifdef USE_CANOPEN_SDO_CLIENT
  // SDO client (one channel, requests run one after another in FIFO order). Every request ends in exactly one
  // callback: accepted ones from loop() / frame processing after the transfer, rejected ones (queue full, invalid
  // arguments, stack not running) synchronously before the call returns false. Requests to our own node id are
  // served from our object dictionary, without CAN frames.

  // value read; expected_size 1/2/4 checks the width (SIZE_MISMATCH), 0 accepts any width up to 4
  bool read_sdo(uint8_t node_id, uint16_t index, uint8_t sub, uint8_t expected_size, SDOCallback callback,
                void *context);
  template<typename T>
  bool read_sdo(uint8_t node_id, uint16_t index, uint8_t sub, SDOCallback callback, void *context) {
    return this->read_sdo(node_id, index, sub, sizeof(T), callback, context);
  }
  // value write of size 1/2/4 bytes (little endian)
  bool write_sdo(uint8_t node_id, uint16_t index, uint8_t sub, uint32_t value, uint8_t size,
                 SDOCallback callback = nullptr, void *context = nullptr);
  template<typename T>
  bool write_sdo(uint8_t node_id, uint16_t index, uint8_t sub, T value, SDOCallback callback = nullptr,
                 void *context = nullptr) {
    static_assert(sizeof(T) <= 4, "SDO values are at most 4 bytes");
    uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(T));
    return this->write_sdo(node_id, index, sub, raw, sizeof(T), callback, context);
  }
  // buffer transfers (strings, domains): segmented, block for more than 32 bytes; reads up to 1024 bytes
  bool read_sdo_data(uint8_t node_id, uint16_t index, uint8_t sub, SDOCallback callback, void *context);
  bool write_sdo_data(uint8_t node_id, uint16_t index, uint8_t sub, const uint8_t *data, size_t len,
                      SDOCallback callback = nullptr, void *context = nullptr);
  // Drops all queued requests of context and aborts its transfer in flight, without callbacks.
  void cancel_sdo(void *context);
#endif
#ifdef USE_CANOPEN_NMT_MASTER
  // NMT command to node_id (0 = all nodes; for our own id or 0 it also applies to us)
  bool send_nmt(uint8_t node_id, NMTCommand command);
#endif

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  // used by the CANopenNode driver (driver_can.cpp)
  canbus::Canbus *get_canbus() { return this->canbus_; }

 protected:
  void on_frame_(uint32_t can_id, bool extended_id, bool rtr, const std::vector<uint8_t> &data);
  void process_();
#ifdef USE_CANOPEN_SDO_CLIENT
  enum class SDOKind : uint8_t { SDO_KIND_READ, SDO_KIND_WRITE, SDO_KIND_READ_DATA, SDO_KIND_WRITE_DATA };
  struct SDORequest {
    uint8_t node_id;
    uint16_t index;
    uint8_t sub;
    SDOKind kind;
    uint8_t size;  // read: expected size (0 = any); write: value size
    uint32_t value;
    std::vector<uint8_t> data;  // WRITE_DATA
    SDOCallback callback;
    void *context;
  };
  static constexpr size_t SDO_QUEUE_SIZE = 16;
  static constexpr size_t SDO_MAX_UPLOAD = 1024;
  bool queue_sdo_(SDORequest &&request);
  void process_sdo_(uint32_t dt_us);
  bool start_sdo_();
  void finish_sdo_(SDOResult result, uint32_t abort_code);
  void abort_sdo_();
#endif
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
  uint16_t response_timeout_ms_{500};
#ifdef USE_CANOPEN_SDO_CLIENT
  // OD 0x1280 (SDO client parameter), set up per request
  uint8_t sdo_cli_highest_sub_{3};
  uint32_t sdo_cli_cob_tx_{0x80000000};
  uint32_t sdo_cli_cob_rx_{0x80000000};
  uint8_t sdo_cli_node_{0};
  std::array<SDORequest, SDO_QUEUE_SIZE> sdo_queue_{};
  size_t sdo_head_{0};
  size_t sdo_count_{0};
  SDORequest sdo_active_{};
  bool sdo_busy_{false};
  size_t sdo_offset_{0};         // WRITE/WRITE_DATA: bytes handed to the client fifo
  std::vector<uint8_t> sdo_rx_;  // READ/READ_DATA: received bytes
  uint8_t sdo_abort_node_{0};    // node of the last cancelled transfer, see ABORT_GUARD_MS
  uint32_t sdo_abort_ms_{0};
#endif
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

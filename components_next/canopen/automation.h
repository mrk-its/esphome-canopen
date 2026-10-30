#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/defines.h"

#include "canopen.h"

namespace esphome {
namespace canopen {

#ifdef USE_CANOPEN_SDO_CLIENT
// canopen.sdo_write: fires the request and continues; on_success / on_error run when it ends
template<typename... Ts> class SDOWriteAction : public Action<Ts...>, public Parented<CANopenComponent> {
 public:
  TEMPLATABLE_VALUE(uint8_t, node_id)
  TEMPLATABLE_VALUE(uint16_t, index)
  TEMPLATABLE_VALUE(uint8_t, subindex)
  TEMPLATABLE_VALUE(uint32_t, value)
  void set_size(uint8_t size) { this->size_ = size; }
  Trigger<> *get_success_trigger() { return &this->success_trigger_; }
  Trigger<SDOResult, uint32_t> *get_error_trigger() { return &this->error_trigger_; }

  void play(const Ts &...x) override {
    this->parent_->write_sdo(this->node_id_.value(x...), this->index_.value(x...), this->subindex_.value(x...),
                             this->value_.value(x...), this->size_, on_reply_, this);
  }

 protected:
  static void on_reply_(void *context, const SDOReply &reply) {
    auto *self = static_cast<SDOWriteAction *>(context);
    if (reply.ok()) {
      self->success_trigger_.trigger();
    } else {
      self->error_trigger_.trigger(reply.result, reply.abort_code);
    }
  }

  uint8_t size_{4};
  Trigger<> success_trigger_;
  Trigger<SDOResult, uint32_t> error_trigger_;
};
#endif

#ifdef USE_CANOPEN_NMT_MASTER
template<typename... Ts> class NMTAction : public Action<Ts...>, public Parented<CANopenComponent> {
 public:
  TEMPLATABLE_VALUE(uint8_t, node_id)
  void set_command(NMTCommand command) { this->command_ = command; }

  void play(const Ts &...x) override { this->parent_->send_nmt(this->node_id_.value(x...), this->command_); }

 protected:
  NMTCommand command_{NMTCommand::NMT_COMMAND_START};
};
#endif

}  // namespace canopen
}  // namespace esphome

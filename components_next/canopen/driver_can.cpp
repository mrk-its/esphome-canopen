// CANopenNode driver functions (301/CO_driver.h) on top of ESPHome canbus. CANptr is the CANopenComponent.
// Everything runs on the main loop: frames are received by CANopenComponent::on_frame_().
#include "canopen.h"

#include <cstring>

namespace esphome {
namespace canopen {

static canbus::Canbus *get_canbus(CO_CANmodule_t *mod) {
  return static_cast<CANopenComponent *>(mod->CANptr)->get_canbus();
}

static bool send_frame(CO_CANmodule_t *mod, CO_CANtx_t *buf) {
  bool rtr = buf->ident & 0x800;
  // the (data, len) overload is in ESPHome dev since after 2026.9 (not in 2026.9.x releases)
  return get_canbus(mod)->send_data(buf->ident & 0x7FF, false, rtr, buf->data, buf->DLC) == canbus::ERROR_OK;
}

}  // namespace canopen
}  // namespace esphome

using esphome::canopen::send_frame;

extern "C" {

void CO_CANsetConfigurationMode(void *CANptr) {}

void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule) { CANmodule->CANnormal = true; }

CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule, void *CANptr, CO_CANrx_t rxArray[], uint16_t rxSize,
                                   CO_CANtx_t txArray[], uint16_t txSize, uint16_t CANbitRate) {
  if (CANmodule == nullptr || CANptr == nullptr || rxArray == nullptr || txArray == nullptr)
    return CO_ERROR_ILLEGAL_ARGUMENT;
  CANmodule->CANptr = CANptr;
  CANmodule->rxArray = rxArray;
  CANmodule->rxSize = rxSize;
  CANmodule->txArray = txArray;
  CANmodule->txSize = txSize;
  CANmodule->CANerrorStatus = 0;
  CANmodule->CANnormal = false;
  CANmodule->useCANrxFilters = false;
  CANmodule->bufferInhibitFlag = false;
  CANmodule->firstCANtxMessage = true;
  CANmodule->CANtxCount = 0;
  CANmodule->errOld = 0;
  memset(rxArray, 0, sizeof(CO_CANrx_t) * rxSize);
  memset(txArray, 0, sizeof(CO_CANtx_t) * txSize);
  return CO_ERROR_NO;
}

void CO_CANmodule_disable(CO_CANmodule_t *CANmodule) {
  if (CANmodule != nullptr)
    CANmodule->CANnormal = false;
}

CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident, uint16_t mask,
                                    bool_t rtr, void *object, void (*CANrx_callback)(void *object, void *message)) {
  if (CANmodule == nullptr || object == nullptr || CANrx_callback == nullptr || index >= CANmodule->rxSize)
    return CO_ERROR_ILLEGAL_ARGUMENT;
  CO_CANrx_t *rx = &CANmodule->rxArray[index];
  rx->object = object;
  rx->CANrx_callback = CANrx_callback;
  rx->ident = (ident & 0x7FF) | (rtr ? 0x800 : 0);
  rx->mask = (mask & 0x7FF) | 0x800;
  return CO_ERROR_NO;
}

CO_CANtx_t *CO_CANtxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident, bool_t rtr,
                               uint8_t noOfBytes, bool_t syncFlag) {
  if (CANmodule == nullptr || index >= CANmodule->txSize)
    return nullptr;
  CO_CANtx_t *tx = &CANmodule->txArray[index];
  tx->ident = (ident & 0x7FF) | (rtr ? 0x800 : 0);
  tx->DLC = noOfBytes;
  tx->bufferFull = false;
  tx->syncFlag = syncFlag;
  return tx;
}

CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer) {
  if (buffer->bufferFull) {
    // previous frame of this buffer is still waiting
    if (!CANmodule->firstCANtxMessage)
      CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
    return CO_ERROR_TX_OVERFLOW;
  }
  if (CANmodule->CANtxCount == 0 && send_frame(CANmodule, buffer)) {
    CANmodule->firstCANtxMessage = false;
    return CO_ERROR_NO;
  }
  // keep it queued (in order behind the others), CO_CANmodule_process() resends it
  buffer->bufferFull = true;
  CANmodule->CANtxCount++;
  return CO_ERROR_NO;
}

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule) {
  for (uint16_t i = 0; i < CANmodule->txSize; i++) {
    CO_CANtx_t *tx = &CANmodule->txArray[i];
    if (tx->bufferFull && tx->syncFlag) {
      tx->bufferFull = false;
      CANmodule->CANtxCount--;
      CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
  }
}

void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
  for (uint16_t i = 0; i < CANmodule->txSize && CANmodule->CANtxCount > 0; i++) {
    CO_CANtx_t *tx = &CANmodule->txArray[i];
    if (!tx->bufferFull)
      continue;
    if (!send_frame(CANmodule, tx))
      break;
    tx->bufferFull = false;
    CANmodule->CANtxCount--;
    CANmodule->firstCANtxMessage = false;
  }
}

}  // extern "C"

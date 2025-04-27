#include "canopen.h"
#include "driver_can.h"

// static CAN_HandleTypeDef DrvCan1;

/******************************************************************************
 * PRIVATE FUNCTIONS
 ******************************************************************************/
namespace esphome {
namespace canopen {

const char *TAG = "can_driver";
const char *TAG_TM = "timer_driver";


static uint32_t prev_us = 0;
static uint64_t total_us = 0;

uint64_t get_micros_u64() {
    uint32_t us = esphome::micros();
    if(prev_us > us) {
      // overflow
      total_us += 0x100000000;
    }
    prev_us = us;
    return total_us | us;
}

void DrvCanInit(void) { ESP_LOGI(TAG, "DrvCanInit"); }

void DrvCanEnable(uint32_t baudrate) { ESP_LOGI(TAG, "DrvCanEnable baudrate: %ld", baudrate); }

char *can_data_str(uint8_t *data, uint8_t len) {
  static char buf[3 * 8 + 1] = "";
  for (int i = 0; i < len; i++) {
    sprintf(buf + i * 3, " %02x", data[i]);
  }
  return buf;
}

int16_t DrvCanSend(CO_IF_FRM *frm) {
  if (!current_canopen) {
    ESP_LOGW(TAG, "no current canopen instance set");
    return -1;
  }
  ESP_LOGV(TAG, "DrvCanSend id: %03lx, len: %d, data:%s", frm->Identifier, frm->DLC, can_data_str(frm->Data, frm->DLC));

  for (auto it = all_instances.begin(); it < all_instances.end(); it++)
    if (*it != current_canopen) {
      // loopback to peer node
      (*it)->recv_frames.push_back(*frm);
    }

  std::vector<uint8_t> data(frm->Data, frm->Data + frm->DLC);

  if (current_canopen->canbus) {
    current_canopen->canbus->send_data(frm->Identifier, false, data);
  }
  return 0;
}

int16_t DrvCanRead(CO_IF_FRM *frm) {
  if (!current_canopen) {
    ESP_LOGW(TAG, "no current canopen instance set");
    return 0;
  }
  if (current_canopen->recv_frames.size() > 0) {
    auto it = current_canopen->recv_frames.begin();
    *frm = *it;
    current_canopen->recv_frames.erase(it);
    ESP_LOGV(TAG, "DrvCanRead id: %03lx, len: %d, data:%s", frm->Identifier, frm->DLC,
             can_data_str(frm->Data, frm->DLC));
    return sizeof(CO_IF_FRM);
  } else {
    ESP_LOGW(TAG, "DrvCanRead: no frame");
  }
  return 0;
}

void DrvCanReset(void) { ESP_LOGI(TAG, "DrvCanReset"); }

void DrvCanClose(void) { ESP_LOGI(TAG, "DrvCanClose"); }

/******************************************************************************
 * PRIVATE FUNCTIONS
 ******************************************************************************/

void DrvTimerInit(uint32_t freq) {
  if (!current_canopen) {
    ESP_LOGW(TAG_TM, "no current canopen instance set");
    return;
  }
  current_canopen -> next_timer_us = 0;
  ESP_LOGV(TAG_TM, "DrvTimerInit, freq: %ld", freq);
}

void DrvTimerStart(void) {
  ESP_LOGV(TAG_TM, "DrvTimerStart, node_id: %ld", current_canopen->node_id);
  /* start the hardware timer counting */
}

uint8_t DrvTimerUpdate(void) {
  if (!current_canopen) {
    ESP_LOGW(TAG_TM, "no current canopen instance set");
    return 0;
  }
  if (!current_canopen->next_timer_us) {
    // timer is stopped
    return 0;
  }

  uint64_t micros = get_micros_u64();
  int64_t dt = current_canopen->next_timer_us - micros;

  ESP_LOGVV(TAG_TM, "DrvTimerUpdate node_id: %d, %lld", current_canopen->node_id, dt);
  return dt > 0 ? 0 : 1;
}

uint32_t DrvTimerDelay(void) {
  if (!current_canopen) {
    ESP_LOGW(TAG_TM, "no current canopen instance set");
    return 0;
  }
  uint64_t micros = get_micros_u64();
  int64_t dt = current_canopen->next_timer_us - micros;
  ESP_LOGV(TAG_TM, "DrvTimerDelay node_id: %ld delay: %lld", current_canopen->node_id, dt);

  // /* return remaining ticks until interrupt occurs */
  return (uint32_t) (dt > 0 ? dt : 0);
}

void DrvTimerReload(uint32_t reload) {
  if (!current_canopen) {
    ESP_LOGW(TAG_TM, "no current canopen instance set");
    return;
  }

  /* configure the next hardware timer interrupt */
  ESP_LOGV(TAG_TM, "DrvTimerReload node_id: %ld, %ld", current_canopen->node_id, reload);

  current_canopen->next_timer_us = get_micros_u64() + reload;
}

void DrvTimerStop(void) {
  if (!current_canopen) {
    ESP_LOGW(TAG_TM, "no current canopen instance set");
    return;
  }
  ESP_LOGV(TAG_TM, "DrvTimerStop, node_id: %ld", current_canopen->node_id);
  current_canopen->next_timer_us = 0;
  /* stop the hardware timer counting */
}

static void DrvNvmInit(void) { ESP_LOGI(TAG, "DrvNvmInit"); }

static uint32_t DrvNvmRead(uint32_t start, uint8_t *buffer, uint32_t size) {
  ESP_LOGI(TAG, "DrvNvmRead, start: %08lx, buf: %p, size: %08lx", start, buffer, size);
  return 0;
}

static uint32_t DrvNvmWrite(uint32_t start, uint8_t *buffer, uint32_t size) {
  ESP_LOGI(TAG, "DrvNvmWrite, start: %08lx, buf: %p, size: %08lx", start, buffer, size);
  return 0;
}

/******************************************************************************
 * PUBLIC VARIABLE
 ******************************************************************************/

const CO_IF_CAN_DRV CanDriver = {DrvCanInit, DrvCanEnable, DrvCanRead, DrvCanSend, DrvCanReset, DrvCanClose};

const CO_IF_TIMER_DRV TimerDriver = {DrvTimerInit, DrvTimerReload, DrvTimerDelay,
                                     DrvTimerStop, DrvTimerStart,  DrvTimerUpdate};

const CO_IF_NVM_DRV NvmDriver = {DrvNvmInit, DrvNvmRead, DrvNvmWrite};

}  // namespace canopen
}  // namespace esphome

struct CO_IF_DRV_T CanOpenStack_Driver = {&esphome::canopen::CanDriver, &esphome::canopen::TimerDriver,
                                          &esphome::canopen::NvmDriver};

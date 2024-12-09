#pragma once

namespace esphome {
namespace canopen {
void DrvCanInit(void);
void DrvCanEnable(uint32_t baudrate);
int16_t DrvCanSend(CO_IF_FRM *frm);
int16_t DrvCanRead(CO_IF_FRM *frm);

void DrvTimerInit(uint32_t freq);
void DrvTimerStart(void);
uint8_t DrvTimerUpdate(void);
uint32_t DrvTimerDelay(void);
void DrvTimerReload(uint32_t reload);
void DrvTimerStop(void);

}  // namespace canopen
}  // namespace esphome

/******************************************************************************
 * INCLUDES
 ******************************************************************************/

#include "co_if.h"

/******************************************************************************
 * PUBLIC SYMBOLS
 ******************************************************************************/

extern const CO_IF_CAN_DRV CanDriver;
extern struct CO_IF_DRV_T CanOpenStack_Driver;

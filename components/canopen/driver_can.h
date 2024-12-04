#pragma once

namespace esphome {
namespace canopen {
    void DrvCanInit(void);
    void DrvCanEnable(uint32_t baudrate);
    int16_t DrvCanSend(CO_IF_FRM *frm);
    int16_t DrvCanRead(CO_IF_FRM *frm);
}
}

/******************************************************************************
 * INCLUDES
 ******************************************************************************/

#include "co_if.h"

/******************************************************************************
 * PUBLIC SYMBOLS
 ******************************************************************************/

extern const CO_IF_CAN_DRV CanDriver;
extern struct CO_IF_DRV_T CanOpenStack_Driver;

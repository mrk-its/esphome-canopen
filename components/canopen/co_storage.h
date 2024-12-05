#pragma once
#include "co_core.h"

namespace esphome {
namespace canopen {

extern CO_OBJ_TYPE StoreCommParams;
#define CO_TSTORE ((CO_OBJ_TYPE *) &esphome::canopen::StoreCommParams)

extern CO_OBJ_TYPE ResetCommParams;
#define CO_TRESET ((CO_OBJ_TYPE *) &esphome::canopen::ResetCommParams)

}  // namespace canopen
}  // namespace esphome

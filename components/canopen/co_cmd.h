#pragma once

#include "co_core.h"
namespace esphome {
namespace canopen {
extern CO_OBJ_TYPE Cmd8;
#define CO_TCMD8 ((CO_OBJ_TYPE *) &Cmd8)

extern CO_OBJ_TYPE Cmd16;
#define CO_TCMD16 ((CO_OBJ_TYPE *) &Cmd16)

extern CO_OBJ_TYPE Cmd32;
#define CO_TCMD32 ((CO_OBJ_TYPE *) &Cmd32)

}
}

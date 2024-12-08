#include "esphome.h"
#include "canopen.h"
#include "co_storage.h"

namespace esphome {
namespace canopen {

uint32_t StoreCommParamsSize(CO_OBJ *obj, CO_NODE *node, uint32_t width) {
  const CO_OBJ_TYPE *uint32 = CO_TUNSIGNED32;
  return uint32->Size(obj, node, width);
}

CO_ERR StoreCommParamsInit(CO_OBJ *obj, CO_NODE *node) { return CO_ERR_NONE; }

CO_ERR StoreCommParamsRead(struct CO_OBJ_T *obj, struct CO_NODE_T *node, void *buffer, uint32_t size) {
  return CO_ERR_OBJ_READ;
}

CO_ERR StoreCommParamsWrite(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
  if (size == 4 && *(uint32_t *) buffer == 0x65766173) {
    ((CanopenNode *)node)->canopen->store_comm_params();
    return CO_ERR_NONE;
  }
  return CO_ERR_OBJ_WRITE;
}

CO_OBJ_TYPE StoreCommParams = {
    StoreCommParamsSize, StoreCommParamsInit, StoreCommParamsRead, StoreCommParamsWrite, NULL,
};

uint32_t ResetCommParamsSize(CO_OBJ *obj, CO_NODE *node, uint32_t width) { return CO_ERR_OBJ_READ; }

CO_ERR ResetCommParamsInit(CO_OBJ *obj, CO_NODE *node) { return CO_ERR_NONE; }

CO_ERR ResetCommParamsRead(struct CO_OBJ_T *obj, struct CO_NODE_T *node, void *buffer, uint32_t size) {
  const CO_OBJ_TYPE *uint32 = CO_TUNSIGNED32;
  return uint32->Read(obj, node, buffer, size);
}

CO_ERR ResetCommParamsWrite(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
  if (size == 4 && *(uint32_t *) buffer == 0x64616F6C) {
    ((CanopenNode *)node)->canopen->reset_comm_params();
    return CO_ERR_NONE;
  }
  return CO_ERR_OBJ_WRITE;
}

CO_OBJ_TYPE ResetCommParams = {ResetCommParamsSize, ResetCommParamsInit, ResetCommParamsRead, ResetCommParamsWrite,
                               NULL};
}  // namespace canopen
}  // namespace esphome
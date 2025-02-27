#include "esphome.h"
#include "canopen.h"
#include "co_cmd.h"

namespace esphome {
namespace canopen {

uint32_t Cmd8Size(CO_OBJ *obj, CO_NODE *node, uint32_t width) {
  const CO_OBJ_TYPE *uint8 = CO_TUNSIGNED8;
  return uint8->Size(obj, node, width);
}

CO_ERR Cmd8Init(CO_OBJ *obj, CO_NODE *node) { return CO_ERR_NONE; }

CO_ERR Cmd8Read(struct CO_OBJ_T *obj, struct CO_NODE_T *node, void *buffer, uint32_t size) {
  ESP_LOGV(TAG, "Cmd8Read: key: %lx", obj->Key);
  const CO_OBJ_TYPE *uint8 = CO_TUNSIGNED8;
  return uint8->Read(obj, node, buffer, size);
}

CO_ERR Cmd8Write(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
  ESP_LOGV(TAG, "Cmd8Write: key: %08lx, val: %02x, size: %lx", obj->Key, *((uint8_t *) buffer), size);

  const CO_OBJ_TYPE *uint8 = CO_TUNSIGNED8;
  CO_ERR result = uint8->Write(obj, node, buffer, size);
  uint32_t index = obj->Key & 0xffffff00;

  auto cmd_handlers = ((CanopenNode *) node)->canopen->can_cmd_handlers;
  auto it = cmd_handlers.find(index);
  if (it != cmd_handlers.end()) {
    it->second(buffer, size);
  }
  return result;
}

CO_OBJ_TYPE Cmd8 = {
    Cmd8Size, Cmd8Init, Cmd8Read, Cmd8Write, 0,
};

uint32_t Cmd16Size(CO_OBJ *obj, CO_NODE *node, uint32_t width) {
  const CO_OBJ_TYPE *uint32 = CO_TUNSIGNED16;
  return uint32->Size(obj, node, width);
}

CO_ERR Cmd16Init(CO_OBJ *obj, CO_NODE *node) { return CO_ERR_NONE; }

CO_ERR Cmd16Read(struct CO_OBJ_T *obj, struct CO_NODE_T *node, void *buffer, uint32_t size) {
  ESP_LOGV(TAG, "Cmd32Read: key: %lx", obj->Key);
  const CO_OBJ_TYPE *uint32 = CO_TUNSIGNED16;
  return uint32->Read(obj, node, buffer, size);
}

CO_ERR Cmd16Write(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
  ESP_LOGV(TAG, "Cmd32Write: key: %08lx, val: %08x, size: %lx", obj->Key, *((uint8_t *) buffer), size);

  const CO_OBJ_TYPE *uint32 = CO_TUNSIGNED16;
  CO_ERR result = uint32->Write(obj, node, buffer, size);
  uint32_t index = obj->Key & 0xffffff00;

  auto cmd_handlers = ((CanopenNode *) node)->canopen->can_cmd_handlers;
  auto it = cmd_handlers.find(index);
  if (it != cmd_handlers.end()) {
    it->second(buffer, size);
  }
  return result;
}

CO_OBJ_TYPE Cmd16 = {
    Cmd16Size, Cmd16Init, Cmd16Read, Cmd16Write, 0,
};

uint32_t Cmd32Size(CO_OBJ *obj, CO_NODE *node, uint32_t width) {
  const CO_OBJ_TYPE *uint32 = CO_TUNSIGNED32;
  return uint32->Size(obj, node, width);
}

CO_ERR Cmd32Init(CO_OBJ *obj, CO_NODE *node) { return CO_ERR_NONE; }

CO_ERR Cmd32Read(struct CO_OBJ_T *obj, struct CO_NODE_T *node, void *buffer, uint32_t size) {
  ESP_LOGV(TAG, "Cmd32Read: key: %lx", obj->Key);
  const CO_OBJ_TYPE *uint32 = CO_TUNSIGNED32;
  return uint32->Read(obj, node, buffer, size);
}

CO_ERR Cmd32Write(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
  ESP_LOGV(TAG, "Cmd32Write: key: %08lx, val: %08x, size: %lx", obj->Key, *((uint8_t *) buffer), size);

  const CO_OBJ_TYPE *uint32 = CO_TUNSIGNED32;
  CO_ERR result = uint32->Write(obj, node, buffer, size);
  uint32_t index = obj->Key & 0xffffff00;

  auto cmd_handlers = ((CanopenNode *) node)->canopen->can_cmd_handlers;
  auto it = cmd_handlers.find(index);
  if (it != cmd_handlers.end()) {
    it->second(buffer, size);
  }
  return result;
}

CO_OBJ_TYPE Cmd32 = {
    Cmd32Size, Cmd32Init, Cmd32Read, Cmd32Write, 0,
};

}  // namespace canopen
}  // namespace esphome
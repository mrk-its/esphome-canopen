#pragma once
#include <vector>
#include "co_core.h"

/******************************************************************************
 * INCLUDES
 ******************************************************************************/

namespace esphome {
namespace canopen {

typedef struct CO_OBJ_T CoObj;

class ObjectDictionary {
 public:
  std::vector<CoObj> od;

  CoObj *find(uint32_t key);
  void add_update(uint32_t key, const CO_OBJ_TYPE *type, CO_DATA data);
  void append(uint32_t key, const CO_OBJ_TYPE *type, CO_DATA data);
  ObjectDictionary(int capacity);
};

}  // namespace canopen
}  // namespace esphome

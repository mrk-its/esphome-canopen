#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#define OD_DEFINITION
#include "CANopen.h"

namespace esphome {
namespace canopen {

// Object dictionary built at runtime (CO_MULTIPLE_OD). Entries are collected with add_*() and turned into the sorted
// OD_entry_t list by finalize(). The value storage is owned by the caller and must outlive the OD.
class ObjectDictionary {
 public:
  void add_var(uint16_t index, OD_attr_t attr, void *data, OD_size_t len);
  // ARRAY: sub0 = *count (read only), subs 1..n = elements of elem_len bytes at data
  void add_array(uint16_t index, OD_attr_t attr, uint8_t *count, void *data, OD_size_t elem_len, uint8_t n);

  // Builds the entry list. Entries can't be added after this.
  void finalize();
  bool finalized() const { return this->finalized_; }

  OD_entry_t *find(uint16_t index) { return OD_find(&this->od_, index); }
  OD_t *od() { return &this->od_; }

 protected:
  void add_entry_(uint16_t index, uint8_t sub_count, uint8_t type, void *obj);

  // deques keep the addresses of their elements stable
  std::deque<OD_obj_var_t> vars_;
  std::deque<OD_obj_array_t> arrays_;
  std::vector<OD_entry_t> entries_;
  OD_t od_{0, nullptr};
  bool finalized_{false};
};

}  // namespace canopen
}  // namespace esphome

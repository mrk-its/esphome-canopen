#include "od.h"

#include <algorithm>

#include "esphome/core/log.h"

namespace esphome {
namespace canopen {

static const char *const TAG = "canopen.od";

static OD_attr_t multi_byte(OD_attr_t attr, OD_size_t len) { return len >= 2 ? attr | ODA_MB : attr; }

void ObjectDictionary::add_entry_(uint16_t index, uint8_t sub_count, uint8_t type, void *obj) {
  if (this->finalized_) {
    ESP_LOGE(TAG, "OD 0x%04x added after init, ignored", index);
    return;
  }
  this->entries_.push_back(OD_entry_t{index, sub_count, type, obj, nullptr});
}

void ObjectDictionary::add_var(uint16_t index, OD_attr_t attr, void *data, OD_size_t len) {
  this->vars_.push_back(OD_obj_var_t{data, multi_byte(attr, len), len});
  this->add_entry_(index, 1, ODT_VAR, &this->vars_.back());
}

void ObjectDictionary::add_array(uint16_t index, OD_attr_t attr, uint8_t *count, void *data, OD_size_t elem_len,
                                 uint8_t n) {
  this->arrays_.push_back(OD_obj_array_t{count, data, ODA_SDO_R, multi_byte(attr, elem_len), elem_len, elem_len});
  this->add_entry_(index, n + 1, ODT_ARR, &this->arrays_.back());
}

void ObjectDictionary::add_record(uint16_t index, std::initializer_list<RecordSub> subs) {
  auto &record = this->records_.emplace_back();
  record.reserve(subs.size());
  for (const auto &s : subs)
    record.push_back(OD_obj_record_t{s.data, s.sub, multi_byte(s.attr, s.len), s.len});
  this->add_entry_(index, record.size(), ODT_REC, record.data());
}

void ObjectDictionary::finalize() {
  std::sort(this->entries_.begin(), this->entries_.end(),
            [](const OD_entry_t &a, const OD_entry_t &b) { return a.index < b.index; });
  this->od_.size = this->entries_.size();
  // OD_find() expects a blank last element
  this->entries_.push_back(OD_entry_t{0, 0, 0, nullptr, nullptr});
  this->od_.list = this->entries_.data();
  this->finalized_ = true;
}

}  // namespace canopen
}  // namespace esphome

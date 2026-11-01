#pragma once

#include <cstdint>
#include <initializer_list>
#include <type_traits>
#include <vector>

#include "esphome/core/defines.h"  // before CANopenNode: clang-tidy gets the CO_CONFIG_* values from it

#define OD_DEFINITION
#include "CANopen.h"

#include "od_handler.h"

namespace esphome::canopen {

// Attributes of OD entries (CANopenNode's ODA_* values)
constexpr uint8_t OD_READ = ODA_SDO_R;
constexpr uint8_t OD_WRITE = ODA_SDO_W;
constexpr uint8_t OD_READ_WRITE = ODA_SDO_RW;
// mappable to a TPDO / RPDO
constexpr uint8_t OD_TPDO = ODA_TPDO;
constexpr uint8_t OD_RPDO = ODA_RPDO;

// Object dictionary built at runtime (CO_MULTIPLE_OD). Entries are collected with add_*() and turned into the sorted
// OD_entry_t list by finalize(). The value storage is owned by the caller and must outlive the OD. All allocation
// happens before finalize() (in setup): the containers only grow until then, and entries refer to their objects by
// position, so the pointers CANopenNode gets are taken once they no longer move.
class ObjectDictionary {
 public:
  struct RecordSub {
    uint8_t sub;
    OD_attr_t attr;
    void *data;
    OD_size_t len;
  };

  void add_var(uint16_t index, OD_attr_t attr, void *data, OD_size_t len);
  // ARRAY: sub0 = *count (read only), subs 1..n = elements of elem_len bytes at data
  void add_array(uint16_t index, OD_attr_t attr, uint8_t *count, void *data, OD_size_t elem_len, uint8_t n);
  // RECORD: subs are looked up by subindex, so sub0 (highest subindex, uint8_t) is passed like the others
  void add_record(uint16_t index, std::initializer_list<RecordSub> subs) {
    this->add_record(index, subs.begin(), subs.size());
  }
  void add_record(uint16_t index, const RecordSub *subs, size_t count);
  // read-only VISIBLE_STRING VAR. Without ODA_STR the SDO server indicates the exact size (short strings go
  // expedited). An empty string is one zero byte: CANopenNode aborts zero-length uploads.
  void add_string(uint16_t index, const char *str);

  // Builds the entry list. Entries can't be added after this. False if an index was added twice.
  bool finalize();
  bool finalized() const { return this->finalized_; }

  OD_entry_t *find(uint16_t index) { return OD_find(&this->od_, index); }
  OD_t *od() { return &this->od_; }

  // --- Handlers (od_handler.h, od_handler.cpp) ---
  // Calls handler->od_read() / od_write() (an OdObject subclass) for every access to an entry added before.
  template<typename T> void set_object(uint16_t index, T *handler) {
    static_assert(std::is_base_of_v<OdObject, T>, "OD handlers derive from canopen::OdObject");
    this->set_extension_(index, OD_extension_t{handler, &read_trampoline<T>, &write_trampoline<T>, {}});
  }
  // Writes a value through the entry's handler, as an SDO download does. Needs write access. After finalize().
  OdResult write(uint16_t index, uint8_t sub, const void *data, size_t len);

 protected:
  // an entry until finalize(): its object and extension by position in vars_ / arrays_ / records_ / extensions_
  struct PendingEntry {
    uint16_t index;
    uint8_t sub_count;
    uint8_t type;
    uint16_t object;
    uint16_t extension;  // position + 1, 0 = none
  };

  bool can_add_(uint16_t index) const;

  // --- Handlers (od_handler.cpp) ---
  void set_extension_(uint16_t index, const OD_extension_t &extension);
  template<typename T>
  static ODR_t read_trampoline(OD_stream_t *stream, void *buf, OD_size_t count, OD_size_t *count_read) {
    size_t done = 0;
    auto ret = static_cast<T *>(stream->object)->od_read(OdStream(stream), buf, count, &done);
    *count_read = done;
    return static_cast<ODR_t>(ret);
  }
  template<typename T>
  static ODR_t write_trampoline(OD_stream_t *stream, const void *buf, OD_size_t count, OD_size_t *count_written) {
    size_t done = 0;
    auto ret = static_cast<T *>(stream->object)->od_write(OdStream(stream), buf, count, &done);
    *count_written = done;
    return static_cast<ODR_t>(ret);
  }

  std::vector<PendingEntry> pending_;  // released by finalize()
  std::vector<OD_obj_var_t> vars_;
  std::vector<OD_obj_array_t> arrays_;
  std::vector<OD_obj_record_t> records_;  // the subs of all records, each record's in a row
  std::vector<OD_extension_t> extensions_;
  std::vector<OD_entry_t> entries_;
  OD_t od_{0, nullptr};
  bool finalized_{false};
};

}  // namespace esphome::canopen

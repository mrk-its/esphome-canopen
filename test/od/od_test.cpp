// Host test of components_next/canopen/od.cpp against CANopenNode's OD interface (run: pytest test/).
#include <cassert>
#include <cstdio>
#include "od.h"

using esphome::canopen::ObjectDictionary;

int main() {
  ObjectDictionary od;
  uint8_t highest = 3, node = 0x05;
  uint32_t cob_tx = 0x605, cob_rx = 0x585;
  uint32_t var = 0x12345678;
  od.add_var(0x1000, ODA_SDO_R, &var, 4);
  od.add_record(0x1280, {{0, ODA_SDO_R, &highest, 1},
                         {1, ODA_SDO_RW, &cob_tx, 4},
                         {2, ODA_SDO_RW, &cob_rx, 4},
                         {3, ODA_SDO_RW, &node, 1}});
  // a second record after the first: deque/vector addresses must stay stable
  uint8_t highest2 = 1;
  uint32_t other = 7;
  od.add_record(0x1281, {{0, ODA_SDO_R, &highest2, 1}, {1, ODA_SDO_RW, &other, 4}});
  od.finalize();

  OD_entry_t *e = od.find(0x1280);
  assert(e != nullptr && e->odObjectType == ODT_REC && e->subEntriesCount == 4);
  uint8_t u8 = 0;
  uint32_t u32 = 0;
  assert(OD_get_u8(e, 0, &u8, true) == ODR_OK && u8 == 3);
  assert(OD_get_u32(e, 1, &u32, true) == ODR_OK && u32 == 0x605);
  assert(OD_get_u32(e, 2, &u32, true) == ODR_OK && u32 == 0x585);
  assert(OD_get_u8(e, 3, &u8, true) == ODR_OK && u8 == 5);
  OD_IO_t io;
  assert(OD_getSub(e, 4, &io, true) == ODR_SUB_NOT_EXIST);
  // multi-byte flag set for len >= 2, not for 1-byte subs
  assert(OD_getSub(e, 1, &io, true) == ODR_OK && (io.stream.attribute & ODA_MB));
  assert(OD_getSub(e, 3, &io, true) == ODR_OK && !(io.stream.attribute & ODA_MB));
  // writes go to the caller's storage
  assert(OD_set_u32(e, 1, 0x80000605, true) == ODR_OK && cob_tx == 0x80000605);

  OD_entry_t *e2 = od.find(0x1281);
  assert(e2 && OD_get_u32(e2, 1, &u32, true) == ODR_OK && u32 == 7);
  assert(od.find(0x1000) && od.find(0x1282) == nullptr);
  std::puts("OK");
  return 0;
}

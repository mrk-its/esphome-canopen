// Host test of decode_heartbeat() / decode_emcy() in components_next/canopen/types.h (run: pytest test/).
#include <cassert>
#include <cstdio>

#include "types.h"

using namespace esphome::canopen;

int main() {
  uint8_t node = 0;
  NMTState state = NMTState::UNKNOWN;
  const uint8_t op[] = {0x05}, preop[] = {0x7F}, boot[] = {0x00}, stopped[] = {0x04}, odd[] = {0x42},
                toggled[] = {0x85};
  assert(decode_heartbeat(0x701, op, 1, node, state) && node == 1 && state == NMTState::OPERATIONAL);
  assert(decode_heartbeat(0x77F, preop, 1, node, state) && node == 0x7F && state == NMTState::PRE_OPERATIONAL);
  assert(decode_heartbeat(0x710, boot, 1, node, state) && state == NMTState::BOOTUP);
  assert(decode_heartbeat(0x710, stopped, 1, node, state) && state == NMTState::STOPPED);
  assert(decode_heartbeat(0x710, odd, 1, node, state) && state == NMTState::UNKNOWN);
  assert(decode_heartbeat(0x710, toggled, 1, node, state) && state == NMTState::OPERATIONAL);
  assert(!decode_heartbeat(0x700, op, 1, node, state));  // node 0 doesn't exist
  assert(!decode_heartbeat(0x780, op, 1, node, state));
  assert(!decode_heartbeat(0x701, op, 0, node, state));
  assert(!decode_heartbeat(0x581, op, 1, node, state));

  Emergency e{};
  const uint8_t emcy[] = {0x10, 0x81, 0x11, 1, 2, 3, 4, 5};
  assert(decode_emcy(0x8A, emcy, 8, node, e) && node == 0x0A && e.error_code == 0x8110 && e.error_register == 0x11);
  assert(e.manufacturer_data[0] == 1 && e.manufacturer_data[4] == 5);
  assert(decode_emcy(0xFF, emcy, 8, node, e) && node == 0x7F);
  assert(!decode_emcy(0x80, emcy, 8, node, e));  // SYNC
  assert(!decode_emcy(0x100, emcy, 8, node, e));
  assert(!decode_emcy(0x81, emcy, 7, node, e));
  assert(!decode_emcy(0x181, emcy, 8, node, e));  // TPDO1

  std::puts("OK");
  return 0;
}

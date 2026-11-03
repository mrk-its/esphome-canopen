// Host test of components/canopen_entities/od_writer_frame.h, built and run by test/test_od_writer_frame.py.
#include <cstdio>
#include <cstdlib>

#include "od_writer_frame.h"

using esphome::canopen_entities::decode_od_writer_frame;
using esphome::canopen_entities::OdWriterFrame;

#define CHECK(cond) \
  do { \
    if (!(cond)) { \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      exit(1); \
    } \
  } while (0)

static constexpr uint8_t NODE = 0x36;

int main() {
  OdWriterFrame frame;

  // old format: target, sub, index, value of 1..4 bytes
  const uint8_t old_1[5] = {NODE, 1, 0x32, 0x20, 0xAA};
  CHECK(decode_od_writer_frame(old_1, 5, NODE, frame));
  CHECK(!frame.dam && frame.index == 0x2032 && frame.sub == 1 && frame.len == 1 && frame.value == old_1 + 4);
  const uint8_t old_4[8] = {NODE, 2, 0x34, 0x12, 1, 2, 3, 4};
  CHECK(decode_od_writer_frame(old_4, 8, NODE, frame));
  CHECK(!frame.dam && frame.index == 0x1234 && frame.sub == 2 && frame.len == 4);
  CHECK(!decode_od_writer_frame(old_1, 4, NODE, frame));      // no value
  CHECK(!decode_od_writer_frame(old_1, 5, NODE + 1, frame));  // another node

  // DAM-MPDO: 0x80 | target, index, sub, 4 bytes
  const uint8_t dam[8] = {0x80 | NODE, 0x32, 0x20, 1, 0xAA, 0, 0, 0};
  CHECK(decode_od_writer_frame(dam, 8, NODE, frame));
  CHECK(frame.dam && frame.index == 0x2032 && frame.sub == 1 && frame.len == 4 && frame.value == dam + 4);
  CHECK(!decode_od_writer_frame(dam, 8, NODE + 1, frame));  // another node
  for (uint8_t len = 0; len < 8; len++)
    CHECK(!decode_od_writer_frame(dam, len, NODE, frame));  // always 8 bytes
  CHECK(!decode_od_writer_frame(dam, 9, NODE, frame));

  // address 0 isn't a node id, in either format
  const uint8_t dam_all[8] = {0x80, 0x32, 0x20, 1, 0, 0, 0, 0};
  CHECK(!decode_od_writer_frame(dam_all, 8, NODE, frame));

  // the same bytes never mean both: a DAM frame's byte 0 can't match in the old format and the other way round
  for (unsigned node = 1; node <= 127; node++) {
    const uint8_t as_dam[8] = {uint8_t(0x80 | node), 0x10, 0x20, 1, 0, 0, 0, 0};
    CHECK(decode_od_writer_frame(as_dam, 8, node, frame) && frame.dam);
    const uint8_t as_old[8] = {uint8_t(node), 1, 0x10, 0x20, 0, 0, 0, 0};
    CHECK(decode_od_writer_frame(as_old, 8, node, frame) && !frame.dam);
  }

  printf("OK\n");
  return 0;
}

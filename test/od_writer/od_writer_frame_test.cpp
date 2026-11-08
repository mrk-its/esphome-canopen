// Host test of components/canopen_entities/od_writer_frame.h, built and run by test/test_od_writer_frame.py.
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "od_writer_frame.h"

using esphome::canopen_entities::decode_od_writer_frame;
using esphome::canopen_entities::encode_dam_mpdo_frame;
using esphome::canopen_entities::encode_old_od_writer_frame;
using esphome::canopen_entities::od_writer_state_index;
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

  // encoders: what is sent decodes to what was meant
  uint8_t out[8];
  encode_dam_mpdo_frame(out, NODE, 0x2032, 3, 0x04030201, 4);
  const uint8_t dam_expected[8] = {0x80 | NODE, 0x32, 0x20, 3, 1, 2, 3, 4};
  CHECK(memcmp(out, dam_expected, 8) == 0);
  CHECK(decode_od_writer_frame(out, 8, NODE, frame));
  CHECK(frame.dam && frame.index == 0x2032 && frame.sub == 3 && frame.len == 4);
  encode_dam_mpdo_frame(out, NODE, 0x2032, 3, 0x04030201, 1);  // padded with zeros, whatever the upper bytes hold
  const uint8_t dam_padded[8] = {0x80 | NODE, 0x32, 0x20, 3, 1, 0, 0, 0};
  CHECK(memcmp(out, dam_padded, 8) == 0);
  encode_old_od_writer_frame(out, NODE, 0x2032, 3, 0x04030201);
  const uint8_t old_expected[8] = {NODE, 3, 0x32, 0x20, 1, 2, 3, 4};
  CHECK(memcmp(out, old_expected, 8) == 0);
  CHECK(decode_od_writer_frame(out, 5, NODE, frame));
  CHECK(!frame.dam && frame.index == 0x2032 && frame.sub == 3 && frame.len == 1 && frame.value[0] == 1);

  // the state object answering a command: commands at 0x2002 + 16 * n, states one below
  uint16_t state = 0;
  CHECK(od_writer_state_index(0x2052, state) && state == 0x2051);
  CHECK(od_writer_state_index(0x2002, state) && state == 0x2001);
  CHECK(od_writer_state_index(0x2FF2, state) && state == 0x2FF1);
  state = 0;
  CHECK(!od_writer_state_index(0x2051, state));  // a state
  CHECK(!od_writer_state_index(0x2050, state));  // metadata
  CHECK(!od_writer_state_index(0x1017, state));  // not an entity
  CHECK(!od_writer_state_index(0x3002, state));  // not an entity, ends in 2 too
  CHECK(!od_writer_state_index(0x1FF2, state));
  CHECK(state == 0);

  printf("OK\n");
  return 0;
}

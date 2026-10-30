// Hardware test of the SDO client and NMT master (step 2 / PR 3), included by test/stm32l4_canopen.yaml.
// Runs once after boot against TEST_NODE and our own node (local transfers). Every check logs PASS / FAIL / SKIP with
// tag "sdo_test"; the summary line is "sdo_test done".
//
// What it changes on TEST_NODE, and how it's undone:
// - its heartbeat period (0x1017): measured from its heartbeat frames first (no measurement -> no write), set to
//   2000 ms, then set back to the measured value. 0x1017 is not read: TEST_NODE runs the old canopen-stack
//   component, which aborts some 0x1017 reads, and a heartbeat time of 0 locks its 0x1017 until a reboot.
// - its NMT state: PRE_OPERATIONAL, then START. No RESET_NODE.
#pragma once

#include <functional>
#include <vector>

#include "esphome/components/canopen/canopen.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace sdo_test {

using namespace esphome;
using namespace esphome::canopen;

static const char *const TAG = "sdo_test";
static constexpr uint8_t TEST_NODE = 0x61;
static constexpr uint8_t ABSENT_NODE = 0x7E;

struct Test {
  CANopenComponent *co{nullptr};
  uint8_t own{0};
  std::vector<std::function<void()>> steps;
  size_t step{0};
  int passed{0};
  int failed{0};
  int skipped{0};
  uint32_t timer_id{1000};
  uint32_t t0{0};
  int counts[3]{};  // callbacks per context in the queue / cancel tests
  // TEST_NODE heartbeats, from on_heartbeat
  uint32_t hb_last_ms{0};
  uint32_t hb_interval_ms{0};
  NMTState hb_state{NMTState::NMT_STATE_UNKNOWN};
  uint16_t hb_orig{0};  // measured period, 0 = no heartbeat seen

  void check(bool ok, const char *what) {
    if (ok) {
      this->passed++;
      ESP_LOGI(TAG, "PASS %s", what);
    } else {
      this->failed++;
      ESP_LOGE(TAG, "FAIL %s", what);
    }
  }
  void skip(const char *what) {
    this->skipped++;
    ESP_LOGW(TAG, "SKIP %s (no heartbeat from node 0x%02x)", what, TEST_NODE);
  }
  void next() {
    if (this->step < this->steps.size()) {
      auto fn = this->steps[this->step++];
      fn();
    } else {
      ESP_LOGI(TAG, "sdo_test done: %d passed, %d failed, %d skipped", this->passed, this->failed, this->skipped);
    }
  }
  void after(uint32_t ms, std::function<void()> &&fn) {
    App.scheduler.set_timeout(this->co, this->timer_id++, ms, std::move(fn));
  }
  // heartbeat period close to expected (the node's timer runs on its own clock)
  bool period_is(uint32_t expected) const {
    return this->hb_interval_ms + 200 > expected && this->hb_interval_ms < expected + 200;
  }
};

static Test test;  // NOLINT

// callback that continues with `then`
struct Pending {
  std::function<void(const SDOReply &)> then;
};
static void on_reply(void *context, const SDOReply &reply) {
  auto *p = static_cast<Pending *>(context);
  auto then = std::move(p->then);
  delete p;
  then(reply);
}
static void *pending(std::function<void(const SDOReply &)> &&then) { return new Pending{std::move(then)}; }

static void count_reply(void *context, const SDOReply &reply) { test.counts[(intptr_t) context]++; }

inline void start(CANopenComponent *co, uint8_t own_node_id) {
  test.co = co;
  test.own = own_node_id;
  co->add_on_heartbeat_callback([](uint8_t node, NMTState state) {
    if (node != TEST_NODE)
      return;
    uint32_t now = millis();
    if (test.hb_last_ms != 0)
      test.hb_interval_ms = now - test.hb_last_ms;
    test.hb_last_ms = now;
    test.hb_state = state;
  });
  auto &s = test.steps;

  // TEST_NODE's heartbeat period: wait for at least two heartbeats (the node is expected at 5 s or less)
  s.push_back([] {
    ESP_LOGI(TAG, "measuring node 0x%02x heartbeat period (12 s)", TEST_NODE);
    test.after(12000, [] {
      if (test.hb_interval_ms > 0) {
        // the period is set in whole 100 ms steps
        test.hb_orig = ((test.hb_interval_ms + 50) / 100) * 100;
        ESP_LOGI(TAG, "node 0x%02x heartbeat period %u ms (measured %u ms)", TEST_NODE, test.hb_orig,
                 (unsigned) test.hb_interval_ms);
      } else {
        ESP_LOGW(TAG, "no heartbeats from node 0x%02x", TEST_NODE);
      }
      test.next();
    });
  });
  s.push_back([] {
    test.co->read_sdo<uint32_t>(TEST_NODE, 0x1018, 1, on_reply, pending([](const SDOReply &r) {
                                  ESP_LOGI(TAG, "node 0x%02x vendor id 0x%08x", TEST_NODE, (unsigned) r.value);
                                  test.check(r.ok() && r.size == 4, "read<uint32_t> 0x1018:1");
                                  test.next();
                                }));
  });
  s.push_back([] {
    test.co->read_sdo<uint8_t>(TEST_NODE, 0x1001, 0, on_reply, pending([](const SDOReply &r) {
                                 test.check(r.ok() && r.size == 1, "read<uint8_t> 0x1001 (error register)");
                                 test.next();
                               }));
  });
  s.push_back([] {
    test.co->read_sdo<uint32_t>(TEST_NODE, 0x1001, 0, on_reply, pending([](const SDOReply &r) {
                                  test.check(r.result == SDOResult::SDO_RESULT_SIZE_MISMATCH && r.size == 1,
                                             "read<uint32_t> of a u8 -> SIZE_MISMATCH");
                                  test.next();
                                }));
  });
  s.push_back([] {
    test.co->read_sdo_data(TEST_NODE, 0x1008, 0, on_reply, pending([](const SDOReply &r) {
                             ESP_LOGI(TAG, "node 0x%02x name: '%.*s'", TEST_NODE, (int) r.len, (const char *) r.data);
                             test.check(r.ok() && r.len > 0, "read_sdo_data 0x1008 (device name)");
                             test.next();
                           }));
  });
  s.push_back([] {
    test.co->read_sdo_data(TEST_NODE, 0x100A, 0, on_reply, pending([](const SDOReply &r) {
                             ESP_LOGI(TAG, "node 0x%02x sw version: '%.*s' (%u bytes)", TEST_NODE, (int) r.len,
                                      (const char *) r.data, (unsigned) r.len);
                             test.check(r.ok() && r.len > 0, "read_sdo_data 0x100A (software version)");
                             test.next();
                           }));
  });
  s.push_back([] {
    test.co->read_sdo(TEST_NODE, 0x5FFF, 0, 0, on_reply, pending([](const SDOReply &r) {
                        test.check(r.result == SDOResult::SDO_RESULT_ABORT && r.abort_code == 0x06020000,
                                   "read 0x5FFF -> ABORT 0x06020000");
                        test.next();
                      }));
  });
  s.push_back([] {
    test.t0 = millis();
    test.co->read_sdo(ABSENT_NODE, 0x1000, 0, 0, on_reply, pending([](const SDOReply &r) {
                        uint32_t dt = millis() - test.t0;
                        ESP_LOGI(TAG, "absent node: %s after %u ms", LOG_STR_ARG(sdo_result_to_string(r.result)), (unsigned) dt);
                        test.check(r.result == SDOResult::SDO_RESULT_TIMEOUT && dt >= 450 && dt < 1000,
                                   "read from absent node -> TIMEOUT after ~500 ms");
                        test.next();
                      }));
  });
  // remote write: heartbeat period 2000 ms, check it in the heartbeats, set the measured period back
  s.push_back([] {
    if (test.hb_orig == 0) {
      test.skip("write 0x1017 = 2000 / restore");
      test.next();
      return;
    }
    test.co->write_sdo<uint16_t>(TEST_NODE, 0x1017, 0, 2000, on_reply, pending([](const SDOReply &r) {
                                   test.check(r.ok(), "write<uint16_t> 0x1017 = 2000");
                                   test.after(5000, [] {
                                     ESP_LOGI(TAG, "heartbeat period now %u ms", (unsigned) test.hb_interval_ms);
                                     test.check(test.period_is(2000), "heartbeats every 2 s");
                                     test.next();
                                   });
                                 }));
  });
  s.push_back([] {
    if (test.hb_orig == 0) {
      test.next();
      return;
    }
    ESP_LOGI(TAG, "restoring node 0x%02x heartbeat period to %u ms", TEST_NODE, test.hb_orig);
    test.co->write_sdo<uint16_t>(TEST_NODE, 0x1017, 0, test.hb_orig, on_reply, pending([](const SDOReply &r) {
                                   test.check(r.ok(), "restore 0x1017");
                                   test.after(2 * test.hb_orig + 500, [] {
                                     ESP_LOGI(TAG, "heartbeat period now %u ms", (unsigned) test.hb_interval_ms);
                                     test.check(test.period_is(test.hb_orig), "heartbeat period restored");
                                     test.next();
                                   });
                                 }));
  });
  // local transfers: our own node, no CAN frames
  s.push_back([] {
    test.co->read_sdo<uint16_t>(test.own, 0x1017, 0, on_reply, pending([](const SDOReply &r) {
                                  test.check(r.ok() && r.as<uint16_t>() == 1000, "local read 0x1017 == 1000");
                                  test.next();
                                }));
  });
  s.push_back([] {
    static const uint8_t v[2] = {0xDC, 0x05};  // 1500; write_sdo_data() does not copy
    test.co->write_sdo_data(test.own, 0x1017, 0, v, 2, on_reply, pending([](const SDOReply &r) {
                              test.check(r.ok(), "local write_sdo_data 0x1017 = 1500");
                              test.next();
                            }));
  });
  s.push_back([] {
    test.co->read_sdo<uint16_t>(test.own, 0x1017, 0, on_reply, pending([](const SDOReply &r) {
                                  test.check(r.ok() && r.as<uint16_t>() == 1500, "local read back 0x1017 == 1500");
                                  test.next();
                                }));
  });
  s.push_back([] {
    test.co->write_sdo<uint16_t>(test.own, 0x1017, 0, 1000, on_reply, pending([](const SDOReply &r) {
                                   test.check(r.ok(), "local restore 0x1017 = 1000");
                                   test.next();
                                 }));
  });
  // queue: 20 requests at once, 16 queued, 4 rejected synchronously, each exactly one callback
  s.push_back([] {
    test.counts[0] = 0;
    int accepted = 0, rejected_sync = 0;
    for (int i = 0; i < 20; i++) {
      int before = test.counts[0];
      if (test.co->read_sdo<uint32_t>(TEST_NODE, 0x1018, 1, count_reply, (void *) 0)) {
        accepted++;
      } else if (test.counts[0] == before + 1) {
        rejected_sync++;
      }
    }
    test.check(accepted == 16 && rejected_sync == 4, "queue: 16 accepted, 4 rejected synchronously");
    test.after(3000, [] {
      test.check(test.counts[0] == 20, "queue: exactly one callback per request (20)");
      test.next();
    });
  });
  // cancel: 5 queued for context 1, one for context 2, cancel 1 -> only context 2 calls back
  s.push_back([] {
    test.counts[1] = test.counts[2] = 0;
    for (int i = 0; i < 5; i++)
      test.co->read_sdo<uint32_t>(TEST_NODE, 0x1018, 1, count_reply, (void *) 1);
    test.co->read_sdo<uint32_t>(TEST_NODE, 0x1018, 1, count_reply, (void *) 2);
    test.co->cancel_sdo((void *) 1);
    test.after(1500, [] {
      test.check(test.counts[1] == 0 && test.counts[2] == 1, "cancel_sdo: queued requests dropped, others run");
      test.next();
    });
  });
  // cancel in flight: start a buffer read, cancel it 1 ms later, then the next read to the node must work at once
  s.push_back([] {
    test.counts[1] = 0;
    test.co->read_sdo_data(TEST_NODE, 0x1008, 0, count_reply, (void *) 1);
    test.after(1, [] {
      test.co->cancel_sdo((void *) 1);
      test.co->read_sdo<uint32_t>(TEST_NODE, 0x1018, 1, on_reply, pending([](const SDOReply &r) {
                                    test.check(test.counts[1] == 0, "cancel_sdo in flight: no callback");
                                    test.check(r.ok(), "read right after cancel in flight");
                                    test.next();
                                  }));
    });
  });
  // NMT: the state in the next heartbeats
  s.push_back([] {
    test.check(test.co->send_nmt(TEST_NODE, NMTCommand::NMT_COMMAND_PRE_OPERATIONAL), "send_nmt PRE_OPERATIONAL");
    if (test.hb_orig == 0) {
      test.skip("node state pre-operational");
      test.next();
      return;
    }
    test.after(test.hb_orig + 500, [] {
      test.check(test.hb_state == NMTState::NMT_STATE_PRE_OPERATIONAL, "heartbeat state pre-operational");
      test.next();
    });
  });
  s.push_back([] {
    test.check(test.co->send_nmt(TEST_NODE, NMTCommand::NMT_COMMAND_START), "send_nmt START");
    if (test.hb_orig == 0) {
      test.skip("node state operational");
      test.next();
      return;
    }
    test.after(test.hb_orig + 500, [] {
      test.check(test.hb_state == NMTState::NMT_STATE_OPERATIONAL, "heartbeat state operational");
      test.next();
    });
  });

  test.next();
}

}  // namespace sdo_test

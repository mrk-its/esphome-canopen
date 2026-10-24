#pragma once
#include <cstdint>
namespace esphome {
extern uint32_t fake_millis;
inline uint32_t millis() { return fake_millis; }
}  // namespace esphome

#pragma once

#include "esphome/core/component.h"

namespace esphome::xiao_tinyuf2_eraser {

class XiaoTinyUf2Eraser : public Component {
 public:
  void dump_config() override;
  // Sets BOOT_SEL to the BOOT0 pin, removes the flash write protection and erases TinyUF2 (0x0-0x8000).
  // Never resets: the board then needs BOOT + RESET to enter the ROM DFU bootloader.
  void erase();
};

}  // namespace esphome::xiao_tinyuf2_eraser

#include "xiao_tinyuf2_eraser.h"

#include <algorithm>

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/linker/linker-defs.h>

#include "esphome/core/log.h"

#include "stm32c5xx_ll_flash.h"

namespace esphome::xiao_tinyuf2_eraser {

ESPHOME_LOG_TAG(TAG, "xiao_tinyuf2_eraser");

static constexpr uintptr_t FLASH_START = 0x08000000;
static constexpr size_t TINYUF2_SIZE = 0x8000;
static constexpr char TINYUF2_MARKER[] = "UF2 Bootloader";
// A cleared WRPxR bit protects a pair of pages.
static constexpr uint32_t WRP_NONE = 0xFFFFFFFF;

static bool boot_sel_is_pin() { return LL_FLASH_OB_GetBoot0SourceSelection(FLASH) == LL_FLASH_OB_BOOT0_BOOTPIN; }

static bool write_protected() { return FLASH->WRP1R_CUR != WRP_NONE || FLASH->WRP2R_CUR != WRP_NONE; }

static bool tinyuf2_blank() {
  const auto *words = reinterpret_cast<const volatile uint32_t *>(FLASH_START);
  for (size_t i = 0; i < TINYUF2_SIZE / sizeof(uint32_t); i++) {
    if (words[i] != 0xFFFFFFFF)
      return false;
  }
  return true;
}

static bool tinyuf2_present() {
  const auto *start = reinterpret_cast<const char *>(FLASH_START);
  const auto *end = start + TINYUF2_SIZE;
  return std::search(start, end, TINYUF2_MARKER, TINYUF2_MARKER + sizeof(TINYUF2_MARKER) - 1) != end;
}

// BOOT_SEL and the write protection in one option byte change. It takes effect at once, without the reset that
// would start TinyUF2 and let it protect itself again.
static bool program_option_bytes() {
  if (boot_sel_is_pin() && !write_protected())
    return true;
  while ((FLASH->SR & FLASH_SR_BSY) != 0U) {
  }
  FLASH->CCR = FLASH_CCR_CLR_OPTCHANGEERR | FLASH_CCR_CLR_PGSERR | FLASH_CCR_CLR_WRPERR | FLASH_CCR_CLR_STRBERR |
               FLASH_CCR_CLR_INCERR | FLASH_CCR_CLR_EOP;
  if (LL_FLASH_IsLocked(FLASH) != 0U) {
    LL_FLASH_SetUnlockKey(FLASH, LL_FLASH_KEY1);
    LL_FLASH_SetUnlockKey(FLASH, LL_FLASH_KEY2);
  }
  if (LL_FLASH_OB_IsLocked(FLASH) != 0U) {
    LL_FLASH_OB_SetUnlockKey(FLASH, LL_FLASH_OB_OPTKEY1);
    LL_FLASH_OB_SetUnlockKey(FLASH, LL_FLASH_OB_OPTKEY2);
  }
  LL_FLASH_OB_SetBoot0SourceSelection(FLASH, LL_FLASH_OB_BOOT0_BOOTPIN);
  LL_FLASH_OB_DisablePageWRP(FLASH, LL_FLASH_BANK_1, LL_FLASH_OB_WRP_PAGE_ALL);
  LL_FLASH_OB_DisablePageWRP(FLASH, LL_FLASH_BANK_2, LL_FLASH_OB_WRP_PAGE_ALL);
  FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT;
  while ((FLASH->SR & FLASH_SR_BSY) != 0U) {
  }
  const uint32_t errors = FLASH->SR & LL_FLASH_FLAG_ERRORS_ALL;
  LL_FLASH_OB_Lock(FLASH);
  LL_FLASH_Lock(FLASH);
  if (errors != 0U || !boot_sel_is_pin() || write_protected()) {
    ESP_LOGE(TAG, "Option bytes not changed: SR errors=0x%08" PRIx32 " BOOT_SEL pin=%s WRP1R=0x%08" PRIx32
                  " WRP2R=0x%08" PRIx32,
             errors, YESNO(boot_sel_is_pin()), FLASH->WRP1R_CUR, FLASH->WRP2R_CUR);
    return false;
  }
  return true;
}

void XiaoTinyUf2Eraser::dump_config() {
  ESP_LOGCONFIG(TAG,
                "TinyUF2 eraser:\n"
                "  TinyUF2 present: %s\n"
                "  BOOT_SEL on BOOT0 pin: %s\n"
                "  WRP1R: 0x%08" PRIx32 ", WRP2R: 0x%08" PRIx32,
                YESNO(tinyuf2_present()), YESNO(boot_sel_is_pin()), FLASH->WRP1R_CUR, FLASH->WRP2R_CUR);
}

void XiaoTinyUf2Eraser::erase() {
  if (reinterpret_cast<uintptr_t>(__rom_region_start) < FLASH_START + TINYUF2_SIZE) {
    ESP_LOGE(TAG, "The app starts at %p, inside the TinyUF2 area; not erasing", __rom_region_start);
    return;
  }
  const bool blank = tinyuf2_blank();
  if (!blank && !tinyuf2_present()) {
    ESP_LOGE(TAG, "0x0-0x%zx holds no TinyUF2; not erasing", TINYUF2_SIZE);
    return;
  }
  if (!program_option_bytes())
    return;
  ESP_LOGI(TAG, "BOOT_SEL is on the BOOT0 pin, no page is write protected");
  if (!blank) {
    const struct device *flash = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    const int err = flash_erase(flash, 0, TINYUF2_SIZE);
    if (err != 0 || !tinyuf2_blank()) {
      ESP_LOGE(TAG, "Erasing TinyUF2 failed: %d", err);
      return;
    }
  }
  ESP_LOGW(TAG, "TinyUF2 erased. Hold BOOT and press RESET, then upload a 'bootloader: mcuboot' build over DFU");
}

}  // namespace esphome::xiao_tinyuf2_eraser

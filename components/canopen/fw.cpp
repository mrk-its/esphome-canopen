#include "esphome.h"
#ifdef USE_CANOPEN_OTA

#include "fw.h"

#define TAG "fw"

namespace esphome {
namespace canopen {

uint32_t FwCtrlSize(CO_OBJ *obj, CO_NODE *node, uint32_t width) {
  const CO_OBJ_TYPE *uint32 = CO_TUNSIGNED32;
  return uint32->Size(obj, node, width);
}

CO_ERR FwCtrlWrite(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
  ESP_LOGI(TAG, "FwCtrlWrite");
  uint32_t command = *((uint32_t *) buffer);
  CO_ERR result = CO_ERR_TYPE_WR;
  result = CO_ERR_NONE;

  if (command == 0xdeadbeef) {
    ESP_LOGI(TAG, "Erasing flash");

    /* erase your firmware region in FLASH here */

    result = CO_ERR_NONE;
  }
  return result;
}

uint32_t FwImageSize(CO_OBJ *obj, CO_NODE *node, uint32_t width) {
  ESP_LOGI(TAG, "FwImageSize: %ld", width);
  Firmware *firmware = (Firmware *) (obj->Data);
  auto domain = &firmware->domain;
  uint32_t size = domain->Size;

  /* allow firmware image smaller or equal to the domain memory area */
  if ((width < size) && (width > 0)) {
    size = width;
    firmware->ota_size = size;
  }
  return size;
}

CO_ERR FwImageInit(CO_OBJ *obj, CO_NODE *node) {
  Firmware *firmware = (Firmware *) (obj->Data);
  auto domain = &firmware->domain;

  ESP_LOGI(TAG, "FwImageInit");
  domain->Offset = 0;
  return (CO_ERR_NONE);
}

CO_ERR FwImageWrite(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size) {
  Firmware *firmware = (Firmware *) (obj->Data);
  auto domain = &firmware->domain;
  CO_ERR result = CO_ERR_TYPE_WR;
  uint32_t address = (uint32_t) domain->Start + domain->Offset;

  if (!((CanopenNode *) node)->canopen->ota) {
    ESP_LOGW(TAG, "FwImageWrite, ota not enabled");
    return CO_ERR_NONE;
  }
  auto ret = ((CanopenNode *) node)->canopen->ota->write((uint8_t *) buffer, size);
  if (ret) {
    ESP_LOGE(TAG, "FwImageWrite, ret: %x", ret);
    return CO_ERR_OBJ_WRITE;
  }
  uint32_t prev = domain->Offset;
  domain->Offset += size;
  if ((prev ^ domain->Offset) & ~1023) {
    uint32_t progress =
        firmware->ota_size > 0 && domain->Offset <= firmware->ota_size ? domain->Offset * 100 / firmware->ota_size : 0;
    ESP_LOGI(TAG, "FwImageWrite, %ld kB (%ld%%)", (domain->Offset >> 10), progress);
  }

  if (domain->Offset >= firmware->ota_size) {
    if (domain->Offset > firmware->ota_size) {
      ESP_LOGW(TAG, "FwImageWrite: too many bytes received: %ld, expected: %ld", domain->Offset, firmware->ota_size);
    }

    char buf[33] = "";
    for (int i = 0; i < 16; i++) {
      sprintf(buf + i * 2, "%02x", firmware->md5[i]);
    }
    ESP_LOGI(TAG, "FwImageWrite: upload complete, md5: %s", buf);

    auto ret = ((CanopenNode *) node)->canopen->ota->end(buf);
    if (ret) {
      ESP_LOGE(TAG, "FwImageWrite, can't end update, ret: %x", ret);
      return CO_ERR_OBJ_WRITE;
    }
  }
  return CO_ERR_NONE;
}

CO_ERR FwImageReset(CO_OBJ *obj, CO_NODE *node, uint32_t para) {
  Firmware *firmware = (Firmware *) (obj->Data);
  auto domain = &firmware->domain;
  ESP_LOGI(TAG, "FwImageReset, size: %lu", firmware->size);
  domain->Offset = 0;
  if (!firmware->size) {
    return CO_ERR_OBJ_WRITE;
  }
  if (!((CanopenNode *) node)->canopen->ota) {
    ESP_LOGW(TAG, "FwImageReset, ota not enabled");
    return CO_ERR_NONE;
  }
  auto ret = ((CanopenNode *) node)->canopen->ota->begin(firmware->size);
  if (ret) {
    ESP_LOGE(TAG, "FwImageReset, can't start OTA, ret: %x", ret);
    return CO_ERR_OBJ_WRITE;
  }
  ESP_LOGI(TAG, "firmware upload started");

  return CO_ERR_NONE;
}
}  // namespace canopen
}  // namespace esphome
#endif

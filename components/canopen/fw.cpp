#include "fw.h"
#include "esphome.h"

#define TAG "fw"

uint32_t FwCtrlSize(CO_OBJ *obj, CO_NODE *node, uint32_t width)
{
  const CO_OBJ_TYPE *uint32 = CO_TUNSIGNED32;
  return uint32->Size(obj, node, width);
}

CO_ERR FwCtrlWrite(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size)
{
  ESP_LOGI(TAG, "FwCtrlWrite");
  uint32_t command = *((uint32_t *)buffer);
  CO_ERR   result  = CO_ERR_TYPE_WR;
  result = CO_ERR_NONE;

  if (command == 0xdeadbeef) {
    ESP_LOGI(TAG, "Erasing flash");



    /* erase your firmware region in FLASH here */

    result = CO_ERR_NONE;
  }
  return result;
}

uint32_t FwImageSize(CO_OBJ *obj, CO_NODE *node, uint32_t width)
{
  Firmware *firmware  = (Firmware*)(obj->Data);
  auto domain = &firmware->domain;
  uint32_t    size   = domain->Size;

  /* allow firmware image smaller or equal to the domain memory area */
  if ((width < size) && (width > 0)) {
    size = width;
  }
  return size;
}

CO_ERR FwImageInit(CO_OBJ *obj, CO_NODE *node)
{
  Firmware *firmware  = (Firmware*)(obj->Data);
  auto domain = &firmware->domain;

  ESP_LOGI(TAG, "FwImageInit");
  domain->Offset = 0;
  return (CO_ERR_NONE);
}

CO_ERR FwImageWrite(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size)
{
  Firmware *firmware  = (Firmware*)(obj->Data);
  auto domain = &firmware->domain;
  CO_ERR      result  = CO_ERR_TYPE_WR;
  uint32_t    address = (uint32_t)domain->Start + domain->Offset;

  /* use your FLASH driver for writing the buffer to given address, e.g.: */
  // success = MyFlashDriverWrite(address, (uint8_t *buffer), size);

  auto ret = firmware->backend->write((uint8_t *)buffer, size);
  if(ret) {
    ESP_LOGE(TAG, "FwImageWrite, ret: %x", ret);
    return CO_ERR_OBJ_WRITE;
  }
  uint32_t prev = domain->Offset;
  domain->Offset += size;
  if((prev ^ domain->Offset) & ~1023) {
    uint32_t progress = firmware->size > 0 && domain->Offset <= firmware->size ? domain->Offset * 100 / firmware->size : 0;
    ESP_LOGI(TAG, "FwImageWrite, %d kB (%d%%)", (domain->Offset >> 10), progress);
  }

  if(domain->Offset == firmware->size) {
    char buf[33] = "";
    for(int i=0; i<16; i++) {
      sprintf(buf + i*2, "%02x", firmware->md5[i]);
    }
    firmware->backend->set_update_md5(buf);
    ret = firmware->backend->end();
    if(ret) {
      ESP_LOGE(TAG, "FwImageWrite, can't end update, ret: %x", ret);
    } else {
      ESP_LOGI(TAG, "FwImageWrite: successfully flashed, rebooting");
      esphome::delay(100);  // NOLINT
      App.safe_reboot();
    }
  }


  result = CO_ERR_NONE;

  return (result);
}

CO_ERR   FwImageReset(CO_OBJ *obj, CO_NODE *node, uint32_t para) {
  Firmware *firmware  = (Firmware*)(obj->Data);
  auto domain = &firmware->domain;
  ESP_LOGI(TAG, "FwImageReset, size: %d", firmware->size);
  domain->Offset = 0;
  if(!firmware->size) {
    return CO_ERR_OBJ_WRITE;
  }
  auto ret = firmware->backend->begin(firmware->size);
  if(ret) {
    ESP_LOGE(TAG, "FwImageReset, can't start OTA, ret: %x", ret);
    return CO_ERR_OBJ_WRITE;
  }
  return CO_ERR_NONE;
}

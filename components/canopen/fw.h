#ifndef FW_H
#define FW_H

#include "esphome/components/ota/ota_backend.h"
#include "esphome/components/ota/ota_component.h"
#include "esphome.h"

#ifdef __cplusplus               /* for compatibility with C++ environments  */

extern "C" {
#endif

/******************************************************************************
* INCLUDES
******************************************************************************/

#include "co_core.h"

#pragma pack(push, 1)
struct Firmware {
    CO_OBJ_DOM domain;
    std::unique_ptr<esphome::ota::OTABackend> backend;
    uint32_t size;
    uint8_t md5[32];
};
#pragma pack(pop)


uint32_t FwCtrlSize (CO_OBJ *obj, CO_NODE *node, uint32_t width);
CO_ERR   FwCtrlWrite(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size);

const CO_OBJ_TYPE FwCtrl = { FwCtrlSize, 0, 0, FwCtrlWrite };

#define FW_CTRL ((CO_OBJ_TYPE*)&FwCtrl)

uint32_t FwImageSize (CO_OBJ *obj, CO_NODE *node, uint32_t width);
CO_ERR   FwImageInit (CO_OBJ *obj, CO_NODE *node);
CO_ERR   FwImageWrite(CO_OBJ *obj, CO_NODE *node, void *buffer, uint32_t size);
CO_ERR   FwImageReset(CO_OBJ *obj, CO_NODE *node, uint32_t para);
const CO_OBJ_TYPE FwImage = { FwImageSize, FwImageInit, 0, FwImageWrite, FwImageReset};

#define FW_IMAGE ((CO_OBJ_TYPE*)&FwImage)

#ifdef __cplusplus               /* for compatibility with C++ environments  */
}
#endif

#endif
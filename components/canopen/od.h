#pragma once
#include <vector>
#include "co_core.h"

/******************************************************************************
 * INCLUDES
 ******************************************************************************/

typedef struct CO_OBJ_T CoObj;

class ObjectDictionary {

static bool _pred(const CoObj &a, const uint32_t key) {
    return CO_GET_DEV(a.Key) < CO_GET_DEV(key);
};


public:
    std::vector<CoObj> od;

    CoObj *find(uint32_t key) {
        auto ret = std::lower_bound(od.begin(), od.end(), key, _pred);
        if(ret!=od.end() && CO_GET_DEV(ret->Key) == CO_GET_DEV(key)) {
            return &*ret;
        }
        return 0;
    }

    void add_update(uint32_t key, const CO_OBJ_TYPE *type, CO_DATA data) {
        uint8_t sub = CO_GET_SUB(key);

        ESP_LOGI("OD", "ad_update %08x %08x %08x, sub: %02x", key, type, data, sub);

        auto ret = std::lower_bound(od.begin(), od.end(), key, _pred);
        if(ret < od.end() && CO_GET_DEV(ret->Key) == CO_GET_DEV(key)) {
            ret->Type = type;
            ret->Data = data;           
            return;
        } else {
            if(od.size() == od.capacity()) {
                ESP_LOGE("OD", "OD is full");
                return;
            }
            od.insert(ret, {key, type, data});
            if(sub) {

                auto idx = CO_GET_IDX(key);
                auto obj = find(CO_DEV(idx, 0));
                if(obj) {
                    if(sub > obj->Data) {
                        ESP_LOGI("OD", "update exsiting sub cnt %08x -> %02x", obj->Data, sub);
                        obj -> Data = sub;
                    } else {
                        ESP_LOGI("OD", "skipping update, sub cnt: %08x > %02x", obj->Data, sub);
                    }
                } else {
                    ESP_LOGI("OD", "inserting new sub cnt");
                    add_update(CO_KEY(idx, 0, CO_OBJ_D___R_), CO_TUNSIGNED8, (CO_DATA) sub);
                }
            }
        }
    }
    void append(uint32_t key, const CO_OBJ_TYPE *type, CO_DATA data) {
        od.push_back({key, type, data});
    }

ObjectDictionary(int capacity) {
    od.reserve(capacity);
    memset(&*od.begin(), 0, capacity * sizeof(CoObj));
};



};
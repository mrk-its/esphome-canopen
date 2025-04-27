#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include <vector>
#include <map>
#include <sstream>

#include "co_if.h"
#include "co_cmd.h"

#ifdef USE_CANOPEN_OTA
#include "ota/ota.h"
#endif

namespace esphome {

#ifdef USE_SENSOR
namespace sensor {
class Sensor;
}
#endif

#ifdef USE_NUMBER
namespace number {
class Number;
}
#endif

#ifdef USE_BINARY_SENSOR
namespace binary_sensor {
class BinarySensor;
}
#endif

#ifdef USE_SWITCH
namespace switch_ {
class Switch;
}
#endif

#ifdef USE_LIGHT
namespace light {
class LightState;
}
#endif

#ifdef USE_COVER
namespace cover {
class Cover;
}
#endif

}  // namespace esphome

namespace esphome {
namespace canopen {

struct TPDO {
  int number;
  bool is_async;
};

class CanopenComponent;

class BaseCanopenEntity {
 public:
  uint32_t entity_id;
  TPDO tpdo;
  BaseCanopenEntity(uint32_t entity_id, TPDO tpdo) {
    this->entity_id = entity_id;
    this->tpdo = tpdo;
  }
  virtual void setup(CanopenComponent *canopen) = 0;
  void od_set_state(CanopenComponent *canopen, uint32_t key, void *state, uint8_t size);
};

#ifdef USE_SENSOR
class SensorEntity : public BaseCanopenEntity {
 public:
  sensor::Sensor *sensor;
  uint8_t size;
  float min_val;
  float max_val;
  SensorEntity(sensor::Sensor *sensor, uint32_t entity_id, TPDO tpdo, uint8_t size = 4, float min_val = 0,
               float max_val = 0)
      : BaseCanopenEntity(entity_id, tpdo) {
    this->sensor = sensor;
    this->size = size;
    this->min_val = min_val;
    this->max_val = max_val;
  }
  void setup(CanopenComponent *canopen) override;
};
#endif

#ifdef USE_NUMBER
class NumberEntity : public BaseCanopenEntity {
 public:
  esphome::number::Number *number;
  uint8_t size;
  float min_val;
  float max_val;
  NumberEntity(esphome::number::Number *number, uint32_t entity_id, TPDO tpdo, uint8_t size = 4, float min_val = 0,
               float max_val = 0)
      : BaseCanopenEntity(entity_id, tpdo) {
    this->number = number;
    this->size = size;
    this->min_val = min_val;
    this->max_val = max_val;
  }
  void setup(CanopenComponent *canopen) override;
};
#endif

#ifdef USE_BINARY_SENSOR
class BinarySensorEntity : public BaseCanopenEntity {
 public:
  binary_sensor::BinarySensor *sensor;
  BinarySensorEntity(binary_sensor::BinarySensor *sensor, uint32_t entity_id, TPDO tpdo)
      : BaseCanopenEntity(entity_id, tpdo) {
    this->sensor = sensor;
  }
  void setup(CanopenComponent *canopen) override;
};
#endif

#ifdef USE_SWITCH
class SwitchEntity : public BaseCanopenEntity {
 public:
  esphome::switch_::Switch *switch_;
  SwitchEntity(esphome::switch_::Switch *switch_, uint32_t entity_id, TPDO tpdo) : BaseCanopenEntity(entity_id, tpdo) {
    this->switch_ = switch_;
  }
  void setup(CanopenComponent *canopen) override;
};
#endif

#ifdef USE_LIGHT
class LightStateEntity : public BaseCanopenEntity {
 public:
  esphome::light::LightState *light;
  LightStateEntity(esphome::light::LightState *light, uint32_t entity_id, TPDO tpdo)
      : BaseCanopenEntity(entity_id, tpdo) {
    this->light = light;
  }
  void setup(CanopenComponent *canopen) override;
};
#endif

#ifdef USE_COVER
class CoverEntity : public BaseCanopenEntity {
 public:
  esphome::cover::Cover *cover;
  CoverEntity(esphome::cover::Cover *cover, uint32_t entity_id, TPDO tpdo) : BaseCanopenEntity(entity_id, tpdo) {
    this->cover = cover;
  }
  void setup(CanopenComponent *canopen) override;
};
#endif

#ifdef USE_ALARM_CONTROL_PANEL
class AlarmEntity : public BaseCanopenEntity {
 public:
  esphome::alarm_control_panel::AlarmControlPanel *alarm;
  AlarmEntity(esphome::alarm_control_panel::AlarmControlPanel *alarm, uint32_t entity_id, TPDO tpdo)
      : BaseCanopenEntity(entity_id, tpdo) {
    this->alarm = alarm;
  }
  void setup(CanopenComponent *canopen) override;
};
#endif

}  // namespace canopen
}  // namespace esphome
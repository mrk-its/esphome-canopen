// Store / restore of the PDO parameters (0x1010 / 0x1011) in the preferences.
#include "canopen.h"

#ifdef USE_CANOPEN_PDO

#include <type_traits>

#include "esphome/core/log.h"

namespace esphome::canopen {

ESPHOME_LOG_TAG(TAG, "canopen");

void CANopenComponent::add_pdo_storage_od_() {
  // stored parameters replace the defaults before the records point into them
  this->load_pdos_();
  this->pdo_storage_ = global_preferences != nullptr && this->has_pdos_();
  if (this->pdo_storage_) {
    auto &od = this->od_;
    od.add_array(0x1010, ODA_SDO_RW, &this->storage_highest_sub_, this->store_caps_.data(), sizeof(uint32_t), 2);
    od.add_array(0x1011, ODA_SDO_RW, &this->storage_highest_sub_, this->restore_caps_.data(), sizeof(uint32_t), 2);
  }
}

bool CANopenComponent::init_pdo_storage_() {
  if (!this->pdo_storage_)
    return true;
  // one entry: the PDO parameters, 0x1010:2 / 0x1011:2 (and sub 1, "all"); the callbacks don't use addr / len
  this->storage_entry_.addr = this;
  this->storage_entry_.len = 0;
  this->storage_entry_.subIndexOD = 2;
  this->storage_entry_.attr = CO_storage_cmd | CO_storage_restore;
  CO_ReturnError_t err =
      CO_storage_init(&this->storage_, this->co_->CANmodule, this->od_.find(0x1010), this->od_.find(0x1011),
                      storage_store, storage_restore, &this->storage_entry_, 1);
  if (err != CO_ERROR_NO) {
    ESP_LOGE(TAG, "CO_storage_init failed: %d", err);
    return false;
  }
  this->storage_.enabled = true;
  return true;
}

// marker of a stored PDO configuration ("PDO1"); "load" (0x1011) clears it
static constexpr uint32_t PDO_STORE_MAGIC = 0x314F4450;

// The preference keys are a hash of the defaults (and the node id), so parameters stored from other defaults (another
// firmware, another PDO layout) are ignored instead of being applied to mappings they weren't made for.
void CANopenComponent::load_pdos_() {
  static_assert(std::is_trivially_copyable_v<PdoParams>, "PdoParams are stored as bytes");
  if (global_preferences == nullptr || !this->has_pdos_())
    return;
  uint32_t key = fnv1a_hash_extend(FNV1_OFFSET_BASIS, "canopen_pdo");
  key = fnv1a_hash_extend(key, this->node_id_);
  for (const auto *pdos : {&this->rpdos_, &this->tpdos_}) {
    key = fnv1a_hash_extend(key, uint32_t(pdos->size()));
    for (const PdoParams &p : *pdos) {
      key = fnv1a_hash_extend(key, p.cob_id);
      key = fnv1a_hash_extend(key, p.transmission_type);
      key = fnv1a_hash_extend(key, p.inhibit_time);
      key = fnv1a_hash_extend(key, p.event_timer);
      key = fnv1a_hash_extend(key, p.map_count);
      for (uint32_t m : p.map)
        key = fnv1a_hash_extend(key, m);
    }
  }
  this->pdo_valid_pref_ = global_preferences->make_preference<uint32_t>(key, true);
  this->rpdo_pref_ = global_preferences->make_preference(this->rpdos_.size() * sizeof(PdoParams), key + 1, true);
  this->tpdo_pref_ = global_preferences->make_preference(this->tpdos_.size() * sizeof(PdoParams), key + 2, true);

  uint32_t magic = 0;
  if (!this->pdo_valid_pref_.load(&magic) || magic != PDO_STORE_MAGIC)
    return;
  // into copies: a failed load leaves the defaults
  std::vector<PdoParams> rpdos = this->rpdos_;
  std::vector<PdoParams> tpdos = this->tpdos_;
  if ((!rpdos.empty() &&
       !this->rpdo_pref_.load(reinterpret_cast<uint8_t *>(rpdos.data()), rpdos.size() * sizeof(PdoParams))) ||
      (!tpdos.empty() &&
       !this->tpdo_pref_.load(reinterpret_cast<uint8_t *>(tpdos.data()), tpdos.size() * sizeof(PdoParams)))) {
    ESP_LOGW(TAG, "Stored PDO parameters incomplete, using the defaults");
    return;
  }
  this->rpdos_ = std::move(rpdos);
  this->tpdos_ = std::move(tpdos);
  ESP_LOGI(TAG, "PDO parameters loaded from storage (0x1011 \"load\" restores the defaults)");
}

bool CANopenComponent::store_pdos_() {
  bool ok = (this->rpdos_.empty() || this->rpdo_pref_.save(reinterpret_cast<const uint8_t *>(this->rpdos_.data()),
                                                           this->rpdos_.size() * sizeof(PdoParams))) &&
            (this->tpdos_.empty() || this->tpdo_pref_.save(reinterpret_cast<const uint8_t *>(this->tpdos_.data()),
                                                           this->tpdos_.size() * sizeof(PdoParams))) &&
            this->pdo_valid_pref_.save(&PDO_STORE_MAGIC) && global_preferences->sync();
  if (ok) {
    ESP_LOGI(TAG, "PDO parameters stored");
  } else {
    ESP_LOGE(TAG, "Storing the PDO parameters failed");
  }
  return ok;
}

bool CANopenComponent::restore_pdo_defaults_() {
  static constexpr uint32_t NONE = 0;
  bool ok = this->pdo_valid_pref_.save(&NONE) && global_preferences->sync();
  if (ok) {
    ESP_LOGI(TAG, "PDO parameters: the defaults apply from the next boot");
  } else {
    ESP_LOGE(TAG, "Restoring the default PDO parameters failed");
  }
  return ok;
}

ODR_t CANopenComponent::storage_store(CO_storage_entry_t * /*entry*/, CO_CANmodule_t *module) {
  return static_cast<CANopenComponent *>(module->CANptr)->store_pdos_() ? ODR_OK : ODR_HW;
}

ODR_t CANopenComponent::storage_restore(CO_storage_entry_t * /*entry*/, CO_CANmodule_t *module) {
  return static_cast<CANopenComponent *>(module->CANptr)->restore_pdo_defaults_() ? ODR_OK : ODR_HW;
}

}  // namespace esphome::canopen

#endif  // USE_CANOPEN_PDO

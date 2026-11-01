// Our own PDOs: configuration (configure_tpdo/rpdo()), the communication and mapping records, processing.
#include "canopen.h"

#ifdef USE_CANOPEN_PDO

#include "esphome/core/log.h"

namespace esphome::canopen {

ESPHOME_LOG_TAG(TAG, "canopen");

bool CANopenComponent::configure_tpdo(uint8_t n, uint32_t cob_id, std::initializer_list<PdoMapping> mappings,
                                      uint16_t event_timer_ms, uint16_t inhibit_time_100us, uint8_t transmission_type) {
  return this->configure_pdo_(this->tpdos_, LOG_STR("TPDO"), n, cob_id, mappings, event_timer_ms, inhibit_time_100us,
                              transmission_type);
}

bool CANopenComponent::configure_rpdo(uint8_t n, uint32_t cob_id, std::initializer_list<PdoMapping> mappings,
                                      uint16_t event_timer_ms, uint8_t transmission_type) {
  return this->configure_pdo_(this->rpdos_, LOG_STR("RPDO"), n, cob_id, mappings, event_timer_ms, 0, transmission_type);
}

bool CANopenComponent::configure_pdo_(std::vector<PdoParams> &pdos, const LogString *kind, uint8_t n, uint32_t cob_id,
                                      std::initializer_list<PdoMapping> mappings, uint16_t event_timer_ms,
                                      uint16_t inhibit_time_100us, uint8_t transmission_type) {
  unsigned bits = 0;
  bool aligned = true;
  for (const auto &m : mappings) {
    bits += m.bits;
    // no CO_CONFIG_PDO_BITWISE_MAPPING: whole bytes
    aligned = aligned && m.bits > 0 && (m.bits % 8) == 0;
  }
  if (this->od_.finalized() || n >= MAX_PDO || mappings.size() > 8 || bits > 64 || !aligned || (cob_id & 0x7FF) == 0) {
    ESP_LOGE(TAG, "%s %u: invalid configuration (or after setup), ignored", LOG_STR_ARG(kind), n);
    return false;
  }
  if (pdos.size() <= n)
    pdos.resize(n + 1);
  PdoParams &p = pdos[n];
  // without mappings the PDO would be invalid (CANopenNode sends an EMCY): disabled, for the host to map later
  p.cob_id = (cob_id & 0x7FF) | (mappings.size() == 0 ? 0x80000000 : 0);
  p.transmission_type = transmission_type;
  p.inhibit_time = inhibit_time_100us;
  p.event_timer = event_timer_ms;
  p.map_count = mappings.size();
  size_t i = 0;
  for (const auto &m : mappings)
    p.map[i++] = (uint32_t(m.index) << 16) | (uint32_t(m.sub) << 8) | m.bits;
  return true;
}

void CANopenComponent::add_pdo_map_(uint16_t index, PdoParams &p) {
  this->od_.add_record(index, {{0, ODA_SDO_RW, &p.map_count, 1},
                               {1, ODA_SDO_RW, &p.map[0], 4},
                               {2, ODA_SDO_RW, &p.map[1], 4},
                               {3, ODA_SDO_RW, &p.map[2], 4},
                               {4, ODA_SDO_RW, &p.map[3], 4},
                               {5, ODA_SDO_RW, &p.map[4], 4},
                               {6, ODA_SDO_RW, &p.map[5], 4},
                               {7, ODA_SDO_RW, &p.map[6], 4},
                               {8, ODA_SDO_RW, &p.map[7], 4}});
}

void CANopenComponent::add_pdo_od_() {
  auto &od = this->od_;
  for (size_t n = 0; n < this->rpdos_.size(); n++) {
    PdoParams &p = this->rpdos_[n];
    od.add_record(0x1400 + n, {{0, ODA_SDO_R, &this->rpdo_comm_highest_, 1},
                               {1, ODA_SDO_RW, &p.cob_id, 4},
                               {2, ODA_SDO_RW, &p.transmission_type, 1},
                               {5, ODA_SDO_RW, &p.event_timer, 2}});
    this->add_pdo_map_(0x1600 + n, p);
  }
  for (size_t n = 0; n < this->tpdos_.size(); n++) {
    PdoParams &p = this->tpdos_[n];
    od.add_record(0x1800 + n, {{0, ODA_SDO_R, &this->tpdo_comm_highest_, 1},
                               {1, ODA_SDO_RW, &p.cob_id, 4},
                               {2, ODA_SDO_RW, &p.transmission_type, 1},
                               {3, ODA_SDO_RW, &p.inhibit_time, 2},
                               {5, ODA_SDO_RW, &p.event_timer, 2}});
    this->add_pdo_map_(0x1A00 + n, p);
  }
}

void CANopenComponent::config_pdo_(CO_config_t &cfg) {
  // CO_CANopenInitPDO() steps from the first record: 0x1400.. / 0x1600.. / 0x1800.. / 0x1A00.. are contiguous
  cfg.CNT_RPDO = this->rpdos_.size();
  cfg.ENTRY_H1400 = this->od_.find(0x1400);
  cfg.ENTRY_H1600 = this->od_.find(0x1600);
  cfg.CNT_TPDO = this->tpdos_.size();
  cfg.ENTRY_H1800 = this->od_.find(0x1800);
  cfg.ENTRY_H1A00 = this->od_.find(0x1A00);
}

bool CANopenComponent::init_pdo_() {
  if (!this->has_pdos_())
    return true;
  uint32_t err_info = 0;
  CO_ReturnError_t err = CO_CANopenInitPDO(this->co_, this->co_->em, this->od_.od(), this->node_id_, &err_info);
  if (err != CO_ERROR_NO) {
    ESP_LOGE(TAG, "CO_CANopenInitPDO failed: %d (OD 0x%04" PRIx32 ")", err, err_info);
    return false;
  }
  return true;
}

void CANopenComponent::process_pdo_(uint32_t dt_us) {
  // no SYNC: event-driven PDOs only
  CO_process_RPDO(this->co_, false, dt_us, nullptr);
  CO_process_TPDO(this->co_, false, dt_us, nullptr);
}

void CANopenComponent::request_tpdo(uint8_t n) {
  if (this->co_ != nullptr && n < this->tpdos_.size())
    CO_TPDOsendRequest(&this->co_->TPDO[n]);
}

bool CANopenComponent::request_tpdo(uint16_t index, uint8_t sub) {
  OD_entry_t *entry = this->od_.finalized() ? this->od_.find(index) : nullptr;
  // the request flags live in the entry's extension (its handler)
  if (entry == nullptr || entry->extension == nullptr || sub >= OD_FLAGS_PDO_SIZE * 8)
    return false;
  OD_requestTPDO(entry, sub);
  return true;
}

}  // namespace esphome::canopen

#endif  // USE_CANOPEN_PDO

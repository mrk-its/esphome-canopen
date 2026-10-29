// SDO client on CANopenNode's CO_SDOclient: one channel, a FIFO request queue, one callback per request.
#include "canopen.h"

#ifdef USE_CANOPEN_SDO_CLIENT

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace canopen {

ESPHOME_LOG_TAG(TAG, "canopen.sdo");

// block transfer only pays off for larger buffers
static constexpr size_t BLOCK_THRESHOLD = 32;
// after an abort, the next request to the same node waits this long, so the server has processed the abort
static constexpr uint32_t ABORT_GUARD_MS = 50;

const char *sdo_result_to_string(SDOResult result) {
  switch (result) {
    case SDOResult::SUCCESS:
      return "success";
    case SDOResult::ABORT:
      return "abort";
    case SDOResult::TIMEOUT:
      return "timeout";
    case SDOResult::SIZE_MISMATCH:
      return "size mismatch";
    case SDOResult::TRANSPORT_ERROR:
      return "transport error";
    case SDOResult::REJECTED:
      return "rejected";
    default:
      return "unknown";
  }
}

static void reject(SDOCallback callback, void *context) {
  if (callback != nullptr)
    callback(context, SDOReply{SDOResult::REJECTED, 0, 0, 0, nullptr, 0});
}

bool CANopenComponent::read_sdo(uint8_t node_id, uint16_t index, uint8_t sub, uint8_t expected_size,
                                SDOCallback callback, void *context) {
  if (expected_size != 0 && expected_size != 1 && expected_size != 2 && expected_size != 4) {
    reject(callback, context);
    return false;
  }
  return this->queue_sdo_(SDORequest{node_id, index, sub, SDOKind::READ, expected_size, 0, {}, callback, context});
}

bool CANopenComponent::write_sdo(uint8_t node_id, uint16_t index, uint8_t sub, uint32_t value, uint8_t size,
                                 SDOCallback callback, void *context) {
  if (size != 1 && size != 2 && size != 4) {
    reject(callback, context);
    return false;
  }
  return this->queue_sdo_(SDORequest{node_id, index, sub, SDOKind::WRITE, size, value, {}, callback, context});
}

bool CANopenComponent::read_sdo_data(uint8_t node_id, uint16_t index, uint8_t sub, SDOCallback callback,
                                     void *context) {
  return this->queue_sdo_(SDORequest{node_id, index, sub, SDOKind::READ_DATA, 0, 0, {}, callback, context});
}

bool CANopenComponent::write_sdo_data(uint8_t node_id, uint16_t index, uint8_t sub, const uint8_t *data, size_t len,
                                      SDOCallback callback, void *context) {
  if (data == nullptr || len == 0) {
    reject(callback, context);
    return false;
  }
  return this->queue_sdo_(SDORequest{node_id, index, sub, SDOKind::WRITE_DATA, 0, 0,
                                     std::vector<uint8_t>(data, data + len), callback, context});
}

bool CANopenComponent::queue_sdo_(SDORequest &&request) {
  if (this->co_ == nullptr || this->is_failed() || request.node_id == 0 || request.node_id > 127 ||
      this->sdo_count_ == SDO_QUEUE_SIZE) {
    ESP_LOGW(TAG, "request to node 0x%02x, 0x%04x:%u rejected", request.node_id, request.index, request.sub);
    reject(request.callback, request.context);
    return false;
  }
  this->sdo_queue_[(this->sdo_head_ + this->sdo_count_) % SDO_QUEUE_SIZE] = std::move(request);
  this->sdo_count_++;
  return true;
}

void CANopenComponent::cancel_sdo(void *context) {
  // keep the other requests in order
  size_t kept = 0;
  for (size_t i = 0; i < this->sdo_count_; i++) {
    SDORequest &r = this->sdo_queue_[(this->sdo_head_ + i) % SDO_QUEUE_SIZE];
    if (r.context == context)
      continue;
    if (kept != i)
      this->sdo_queue_[(this->sdo_head_ + kept) % SDO_QUEUE_SIZE] = std::move(r);
    kept++;
  }
  this->sdo_count_ = kept;
  if (this->sdo_busy_ && this->sdo_active_.context == context)
    this->abort_sdo_();
}

bool CANopenComponent::start_sdo_() {
  CO_SDOclient_t *cli = this->co_->SDOclient;
  SDORequest &r = this->sdo_active_;
  uint8_t node = r.node_id;
  if (CO_SDOclient_setup(cli, 0x600 + node, 0x580 + node, node) != CO_SDO_RT_ok_communicationEnd)
    return false;
  this->sdo_offset_ = 0;
  this->sdo_rx_.clear();
  switch (r.kind) {
    case SDOKind::READ:
      return CO_SDOclientUploadInitiate(cli, r.index, r.sub, this->response_timeout_ms_, false) ==
             CO_SDO_RT_ok_communicationEnd;
    case SDOKind::READ_DATA:
      return CO_SDOclientUploadInitiate(cli, r.index, r.sub, this->response_timeout_ms_, true) ==
             CO_SDO_RT_ok_communicationEnd;
    case SDOKind::WRITE: {
      uint8_t buf[4] = {(uint8_t) r.value, (uint8_t) (r.value >> 8), (uint8_t) (r.value >> 16),
                        (uint8_t) (r.value >> 24)};
      if (CO_SDOclientDownloadInitiate(cli, r.index, r.sub, r.size, this->response_timeout_ms_, false) !=
          CO_SDO_RT_ok_communicationEnd)
        return false;
      this->sdo_offset_ = CO_SDOclientDownloadBufWrite(cli, buf, r.size);
      return this->sdo_offset_ == r.size;
    }
    case SDOKind::WRITE_DATA:
      if (CO_SDOclientDownloadInitiate(cli, r.index, r.sub, r.data.size(), this->response_timeout_ms_,
                                       r.data.size() > BLOCK_THRESHOLD) != CO_SDO_RT_ok_communicationEnd)
        return false;
      this->sdo_offset_ = CO_SDOclientDownloadBufWrite(cli, r.data.data(), r.data.size());
      return true;
  }
  return false;
}

void CANopenComponent::process_sdo_(uint32_t dt_us) {
  if (!this->sdo_busy_) {
    if (this->sdo_count_ == 0)
      return;
    if (this->sdo_queue_[this->sdo_head_].node_id == this->sdo_abort_node_) {
      if (millis() - this->sdo_abort_ms_ < ABORT_GUARD_MS)
        return;
      this->sdo_abort_node_ = 0;
    }
    this->sdo_active_ = std::move(this->sdo_queue_[this->sdo_head_]);
    this->sdo_head_ = (this->sdo_head_ + 1) % SDO_QUEUE_SIZE;
    this->sdo_count_--;
    this->sdo_busy_ = true;
    if (!this->start_sdo_()) {
      this->finish_sdo_(SDOResult::TRANSPORT_ERROR, 0);
      return;
    }
    dt_us = 0;
  }

  CO_SDOclient_t *cli = this->co_->SDOclient;
  SDORequest &r = this->sdo_active_;
  CO_SDO_abortCode_t abort = CO_SDO_AB_NONE;
  CO_SDO_return_t ret;
  if (r.kind == SDOKind::READ || r.kind == SDOKind::READ_DATA) {
    ret = CO_SDOclientUpload(cli, dt_us, false, &abort, nullptr, nullptr, nullptr);
    if (ret == CO_SDO_RT_uploadDataBufferFull || ret == CO_SDO_RT_ok_communicationEnd) {
      uint8_t buf[64];
      size_t n;
      while ((n = CO_SDOclientUploadBufRead(cli, buf, sizeof(buf))) > 0)
        this->sdo_rx_.insert(this->sdo_rx_.end(), buf, buf + n);
      if (this->sdo_rx_.size() > SDO_MAX_UPLOAD) {
        abort = CO_SDO_AB_OUT_OF_MEM;
        CO_SDOclientUpload(cli, 0, true, &abort, nullptr, nullptr, nullptr);
        this->finish_sdo_(SDOResult::ABORT, abort);
        return;
      }
    }
  } else {
    if (r.kind == SDOKind::WRITE_DATA && this->sdo_offset_ < r.data.size()) {
      this->sdo_offset_ +=
          CO_SDOclientDownloadBufWrite(cli, r.data.data() + this->sdo_offset_, r.data.size() - this->sdo_offset_);
    }
    bool partial = r.kind == SDOKind::WRITE_DATA && this->sdo_offset_ < r.data.size();
    ret = CO_SDOclientDownload(cli, dt_us, false, partial, &abort, nullptr, nullptr);
  }

  if (ret == CO_SDO_RT_ok_communicationEnd) {
    this->finish_sdo_(SDOResult::SUCCESS, 0);
  } else if (ret < 0) {
    if (abort == CO_SDO_AB_TIMEOUT) {
      this->finish_sdo_(SDOResult::TIMEOUT, abort);
    } else if (ret == CO_SDO_RT_endedWithServerAbort || ret == CO_SDO_RT_endedWithClientAbort) {
      this->finish_sdo_(SDOResult::ABORT, abort);
    } else {
      this->finish_sdo_(SDOResult::TRANSPORT_ERROR, abort);
    }
  }
}

void CANopenComponent::finish_sdo_(SDOResult result, uint32_t abort_code) {
  CO_SDOclientClose(this->co_->SDOclient);
  SDORequest r = std::move(this->sdo_active_);
  this->sdo_busy_ = false;

  SDOReply reply{result, abort_code, 0, 0, nullptr, 0};
  if (result == SDOResult::SUCCESS) {
    if (r.kind == SDOKind::READ) {
      size_t len = this->sdo_rx_.size();
      if (len == 0 || len > 4 || (r.size != 0 && len != r.size)) {
        reply.result = SDOResult::SIZE_MISMATCH;
      } else {
        for (size_t i = 0; i < len; i++)
          reply.value |= (uint32_t) this->sdo_rx_[i] << (8 * i);
      }
      reply.size = len;
    } else if (r.kind == SDOKind::READ_DATA) {
      reply.data = this->sdo_rx_.data();
      reply.len = this->sdo_rx_.size();
    } else if (r.kind == SDOKind::WRITE) {
      reply.value = r.value;
      reply.size = r.size;
    }
  }
  if (reply.result != SDOResult::SUCCESS) {
    ESP_LOGW(TAG, "node 0x%02x, 0x%04x:%u: %s (abort 0x%08" PRIx32 ")", r.node_id, r.index, r.sub,
             sdo_result_to_string(reply.result), abort_code);
  }
  // the slot is free now, so the callback may queue the next request
  if (r.callback != nullptr)
    r.callback(r.context, reply);
}

void CANopenComponent::abort_sdo_() {
  CO_SDOclient_t *cli = this->co_->SDOclient;
  SDORequest &r = this->sdo_active_;
  CO_SDO_abortCode_t abort = CO_SDO_AB_GENERAL;
  CO_SDO_return_t ret;
  if (r.kind == SDOKind::READ || r.kind == SDOKind::READ_DATA) {
    ret = CO_SDOclientUpload(cli, 0, true, &abort, nullptr, nullptr, nullptr);
  } else {
    ret = CO_SDOclientDownload(cli, 0, true, false, &abort, nullptr, nullptr);
  }
  // CANopenNode sends the abort frame only if its TX buffer is free; otherwise the server would stay in the transfer
  // and reject the next request (0x05040001). Requests to our own node have no server to tell.
  if (ret != CO_SDO_RT_endedWithClientAbort && r.node_id != this->node_id_) {
    const uint8_t frame[8] = {0x80, (uint8_t) r.index, (uint8_t) (r.index >> 8), r.sub, 0x00, 0x00, 0x00, 0x08};
    this->canbus_->send_data(0x600 + r.node_id, false, false, frame, sizeof(frame));
  }
  ESP_LOGD(TAG, "node 0x%02x, 0x%04x:%u: cancelled (client abort %s)", r.node_id, r.index, r.sub,
           ret == CO_SDO_RT_endedWithClientAbort ? "sent" : "sent directly");
  CO_SDOclientClose(cli);
  this->sdo_abort_node_ = r.node_id;
  this->sdo_abort_ms_ = millis();
  this->sdo_active_ = SDORequest{};
  this->sdo_busy_ = false;
}

}  // namespace canopen
}  // namespace esphome

#endif  // USE_CANOPEN_SDO_CLIENT

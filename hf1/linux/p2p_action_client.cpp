#include "p2p_action_client.h"
#include <string.h>
#include <sstream>
#include "logger_interface.h"
#include <mutex>

bool P2PActionClientHandlerBase::Request(int payload_length, const uint8_t *payload, P2PPriority priority, bool guarantee_delivery = false) {
  if (!allows_concurrent_requests_) {
    ASSERTM(state_ == kIdle, "Action cannot be requested again before getting a reply.");
  }

  // Protect with a mutex as this will be called from a different thread than Run().
  std::lock_guard<std::mutex> guard(mutex_);

  request_length_ = payload_length;
  if (payload_length > 0 && payload != nullptr) {
    memcpy(request_payload_, payload, payload_length);
  }
  request_priority_ = priority;
  request_guarantee_delivery_ = guarantee_delivery;
  state_ = kSendingRequest;
}

bool P2PActionClientHandlerBase::Cancel(P2PPriority priority, bool guarantee_delivery = false) {
  if (!allows_concurrent_requests_) {
    ASSERTM(state_ == kWaitingForReply, "Cannot cancel an action that has not been requested.");
  }

  // Protect with a mutex as this will be called from a different thread than Run().
  std::lock_guard<std::mutex> guard(mutex_);

  if (state_ == kSendingRequest) {
    state_ = kIdle;
    return true;
  }
  state_ = kSendingCancellation;
}

void P2PActionClientHandlerBase::Run() {
  // Protect with a mutex as this will be called from a different thread than Request().
  std::lock_guard<std::mutex> guard(mutex_);

  switch(state_) {
    case kSendingRequest: {
      auto maybe_new_packet = p2p_stream_.output().NewPacket(request_priority_);
      if (!maybe_new_packet.ok()) {
        break;
      }
      maybe_new_packet->length() = sizeof(P2PApplicationPacketHeader) + request_length_;
      P2PApplicationPacketHeader *header = reinterpret_cast<P2PApplicationPacketHeader *>(maybe_new_packet->content());
      header->action = action_;
      header->stage = P2PActionStage::kRequest;
      memcpy(maybe_new_packet->content() + sizeof(P2PApplicationPacketHeader), request_payload_, request_length_);
      p2p_stream_.output().Commit(request_priority_, request_guarantee_delivery_);
      state_ = allows_concurrent_requests_ ? kIdle : kWaitingForReply;
      break;
    }
    case kSendingCancellation: {
      auto maybe_new_packet = p2p_stream_.output().NewPacket(request_priority_);
      if (!maybe_new_packet.ok()) {
        break;
      }
      maybe_new_packet->length() = sizeof(P2PApplicationPacketHeader);
      P2PApplicationPacketHeader *header = reinterpret_cast<P2PApplicationPacketHeader *>(maybe_new_packet->content());
      header->action = action_;
      header->stage = P2PActionStage::kCancel;
      p2p_stream_.output().Commit(request_priority_, request_guarantee_delivery_);
      // TODO: the action server should send a packet acknowleging the cancellation.
      state_ = allows_concurrent_requests_ ? kIdle : kWaitingForReply;
      break;
    }
    case kWaitingForReply:
      break;
  }
}

P2PActionClient::P2PActionClient(P2PPacketStreamLinux *p2p_stream, const TimerInterface *system_timer)
  : p2p_stream_(*ASSERT_NOT_NULL(p2p_stream)), system_timer_(*ASSERT_NOT_NULL(system_timer)) {
  for (int i = 0; i < sizeof(handlers_) / sizeof(handlers_[0]); ++i) {
    handlers_[i] = nullptr;
  }
}

void P2PActionClient::Run() {
  // Send any pending request or cancellations. 
  for (int i = 0; i < sizeof(handlers_) / sizeof(handlers_[0]); ++i) {
    auto *handler = handlers_[i];
    if (handler == nullptr) {
      continue;
    }
    handler->Run();
  }

  // Process new packets.
  const auto &maybe_packet = p2p_stream_.input().OldestPacket();
  if (!maybe_packet.ok()) {
    return;
  }  

  const auto *header = reinterpret_cast<const P2PApplicationPacketHeader *>(maybe_packet->content());  
  if (header->action >= P2PAction::kCount) {
    std::ostringstream oss;
    oss << "Unknown action " << header->action << ".";
    LOG_ERROR(oss.str().c_str());
  }

  P2PActionClientHandlerBase *handler = handlers_[header->action];
  if (handler == nullptr) {
    std::ostringstream oss;
    oss << "No handler installed for action " << header->action << ".";
    LOG_WARNING(oss.str().c_str());    
    return;
  }

  const uint8_t *payload = maybe_packet->content() + sizeof(P2PApplicationPacketHeader);
  switch(header->stage) {
    case P2PActionStage::kReply:
      if (!handler->is_request_in_progress()) {
        std::ostringstream oss;
        oss << "Reply for action " << header->action << " when it had not been requested.";
        LOG_WARNING(oss.str().c_str());    
      }
      handler->OnReply(maybe_packet->length() - sizeof(P2PApplicationPacketHeader), payload);
      break;
    case P2PActionStage::kProgress:
      if (!handler->is_request_in_progress()) {
        std::ostringstream oss;
        oss << "Progress for action " << header->action << " when it had not been requested.";
        LOG_WARNING(oss.str().c_str());    
      }
      handler->OnProgress(maybe_packet->length() - sizeof(P2PApplicationPacketHeader), payload);
      break;
    default: {
      std::ostringstream oss;
      oss << "Unsupported stage " << header->stage << " for action " << header->action << ".";
      LOG_ERROR(oss.str().c_str());
      break;
    }
  }

  p2p_stream_.input().Consume(maybe_packet->priority());
}
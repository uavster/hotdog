#include "p2p_action_client.h"
#include <string.h>
#include <sstream>
#include "logger_interface.h"
#include <mutex>
#include <iostream>

bool P2PActionClientHandlerBase::Request(int payload_length, const void *payload, std::optional<P2PPriority> priority, std::optional<bool> guarantee_delivery) {
  ASSERTM(allows_concurrent_requests_ || state_ == kIdle, 
    "Action cannot be requested again before getting a reply.");

  std::lock_guard<std::mutex> guard(p2p_mutex_);

  auto maybe_new_packet = p2p_stream_.output().NewPacket(priority.has_value() ? *priority : priority_);
  if (!maybe_new_packet.ok()) {
    return false;
  }
  maybe_new_packet->length() = sizeof(P2PApplicationPacketHeader) + payload_length;
  P2PApplicationPacketHeader *header = reinterpret_cast<P2PApplicationPacketHeader *>(maybe_new_packet->content());
  header->action = action_;
  header->stage = P2PActionStage::kRequest;
  header->request_id = ++current_request_id_;
  memcpy(maybe_new_packet->content() + sizeof(P2PApplicationPacketHeader), payload, payload_length);
  p2p_stream_.output().Commit(maybe_new_packet->priority(), guarantee_delivery.has_value() ? *guarantee_delivery : guarantee_delivery_);

  state_ = allows_concurrent_requests_ ? kIdle : kWaitingForResponse;
  return true;
}

bool P2PActionClientHandlerBase::Cancel(std::optional<P2PPriority> priority, std::optional<bool> guarantee_delivery) {
  ASSERTM(allows_concurrent_requests_ || state_ == kWaitingForResponse, 
    "Cannot cancel an action that has not been requested.");

  // Protect with a mutex as this will be called from a different thread than Run().
  std::lock_guard<std::mutex> guard(p2p_mutex_);

  auto maybe_new_packet = p2p_stream_.output().NewPacket(priority.has_value() ? *priority : priority_);
  if (!maybe_new_packet.ok()) {
    return false;
  }
  maybe_new_packet->length() = sizeof(P2PApplicationPacketHeader);
  P2PApplicationPacketHeader *header = reinterpret_cast<P2PApplicationPacketHeader *>(maybe_new_packet->content());
  header->action = action_;
  header->stage = P2PActionStage::kCancel;
  header->request_id = current_request_id_;
  p2p_stream_.output().Commit(maybe_new_packet->priority(), guarantee_delivery.has_value() ? *guarantee_delivery : guarantee_delivery_);

  state_ = kIdle;
  return true;
}

bool P2PActionClientHandlerBase::in_progress() const { 
  return !allows_concurrent_requests_ && state_ != kIdle;
}

void P2PActionClientHandlerBase::OnReply(int payload_length, const void *payload) {
  // It is possible to get a reply after cancelling the action, if the cancellation arrives
  // at the other end after it has sent the reply packet.
  state_ = kIdle;  
}

void P2PActionClientHandlerBase::OnProgress(int payload_length, const void *payload) {
  // It is possible to get a progress packet after cancelling the action, if the 
  // cancellation arrives at the other end after it has sent the progress packet.
}

P2PActionClient::P2PActionClient(P2PPacketStreamLinux *p2p_stream, const TimerInterface *system_timer)
  : p2p_stream_(*ASSERT_NOT_NULL(p2p_stream)), system_timer_(*ASSERT_NOT_NULL(system_timer)) {
  for (int i = 0; i < sizeof(handlers_) / sizeof(handlers_[0]); ++i) {
    handlers_[i] = nullptr;
  }
}

void P2PActionClient::Register(P2PActionClientHandlerBase *handler) {
  ASSERT(handler->action() < sizeof(handlers_) / sizeof(handlers_[0]));
  ASSERT(handlers_[handler->action()] == NULL);
  handlers_[handler->action()] = handler;
}

void P2PActionClient::Run() {
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
    p2p_stream_.input().Consume(maybe_packet->priority());
    return;
  }

  P2PActionClientHandlerBase *handler = handlers_[header->action];
  if (handler == nullptr) {
    std::ostringstream oss;
    oss << "No handler installed for action " << header->action << ".";
    LOG_WARNING(oss.str().c_str());    
    p2p_stream_.input().Consume(maybe_packet->priority());
    return;
  }

  if (header->request_id != handler->current_request_id()) {
    // This is a response from a previous action that was cancelled.
    p2p_stream_.input().Consume(maybe_packet->priority());
    return;
  }

  const uint8_t *payload = maybe_packet->content() + sizeof(P2PApplicationPacketHeader);
  switch(header->stage) {
    case P2PActionStage::kReply:
      handler->OnReply(maybe_packet->length() - sizeof(P2PApplicationPacketHeader), payload);
      break;
    case P2PActionStage::kProgress:
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
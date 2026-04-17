#include "p2p_action_client.h"
#include <string.h>
#include <sstream>
#include "logger_interface.h"
#include <mutex>
#include <iostream>

Status P2PActionClientHandlerBase::Request(int payload_length, const void *payload, std::optional<P2PPriority> priority, std::optional<bool> guarantee_delivery) {
  // Protect with a mutex as this will be called from a different thread than Run().
  std::lock_guard<std::mutex> guard(p2p_mutex_);

  if (!allows_concurrent_requests_ && state_ != kIdle) {
    return Status::kExistsError;
  }

  auto maybe_new_packet = p2p_stream_.output().NewPacket(priority.has_value() ? *priority : priority_);
  if (!maybe_new_packet.ok()) {
    return Status::kUnavailableError;
  }
  maybe_new_packet->length() = sizeof(P2PApplicationPacketHeader) + payload_length;
  P2PApplicationPacketHeader *header = reinterpret_cast<P2PApplicationPacketHeader *>(maybe_new_packet->content());
  header->action = action_;
  header->stage = P2PActionStage::kRequest;
  header->request_id = ++current_request_id_;
  memcpy(maybe_new_packet->content() + sizeof(P2PApplicationPacketHeader), payload, payload_length);
  if (action_ == 5) { std::cout << "eoeoeoeo" << std::endl; };
  p2p_stream_.output().Commit(maybe_new_packet->priority(), guarantee_delivery.has_value() ? *guarantee_delivery : guarantee_delivery_);

  state_ = allows_concurrent_requests_ ? kIdle : kWaitingForResponse;
  return Status::kSuccess;
}

Status P2PActionClientHandlerBase::Cancel(std::optional<P2PPriority> priority, std::optional<bool> guarantee_delivery) {
  // Protect with a mutex as this will be called from a different thread than Run().
  std::lock_guard<std::mutex> guard(p2p_mutex_);

  if (!allows_concurrent_requests_) {
    if (state_ == kIdle) {
      return Status::kDoesNotExistError;
    } else {
      if (state_ == kCancelling) {
        // If we're already trying to cancel the action, tell the caller that they can move on,
        // as the client will no longer be routing replies or progress to anybody.
        return Status::kSuccess;
      }
    }
  }

  auto maybe_new_packet = p2p_stream_.output().NewPacket(priority.has_value() ? *priority : priority_);
  if (!maybe_new_packet.ok()) {
    return Status::kUnavailableError;
  }
  maybe_new_packet->length() = sizeof(P2PApplicationPacketHeader);
  P2PApplicationPacketHeader *header = reinterpret_cast<P2PApplicationPacketHeader *>(maybe_new_packet->content());
  header->action = action_;
  header->stage = P2PActionStage::kCancel;
  header->request_id = current_request_id_;
  p2p_stream_.output().Commit(maybe_new_packet->priority(), guarantee_delivery.has_value() ? *guarantee_delivery : guarantee_delivery_);

  state_ = kIdle;
  return Status::kSuccess;
}

bool P2PActionClientHandlerBase::in_progress() const { 
  // No need to lock p2p_mutex_ because state_ is atomic.
  return !allows_concurrent_requests_ && state_ != kIdle;
}

void P2PActionClientHandlerBase::OnReply(int payload_length, const void *payload) {
  // It is fine to modify state_ because this function is called only from Run(), which the caller calls with p2p_mutex_ locked.
  state_ = kIdle;  
}

void P2PActionClientHandlerBase::OnProgress(int payload_length, const void *payload) {
}

void P2PActionClientHandlerBase::OnOtherEndStarted() {
  // If the other has restarted, the action might be gone and we might never get a reply. 
  // However, if the action request was in the output buffer, the other end might get it
  // after sending the handshake packet that triggered this callback, so we need to ensure
  // that a cancellation is sent before marking it as cancelled locally. We the state as
  // kCancelling, so Run() ensures that a cancellation packet is sent, after which the 
  // action will be marked as kIdle, ready to be used again.
  if (state_ == kWaitingForResponse) {
    state_ = kCancelling;
  }
}

void P2PActionClientHandlerBase::OnAbort() {
}

void P2PActionClientHandlerBase::Run() {
  // This is called from the action client's Run(), so p2p_mutex_ is locked.

  if (state_ != kCancelling) { return; }

  // The action is being cancelled locally. Try to send the cancellation packet.
  auto maybe_new_packet = p2p_stream_.output().NewPacket(priority_);
  if (!maybe_new_packet.ok()) {
    return;    
  }

  maybe_new_packet->length() = sizeof(P2PApplicationPacketHeader);
  P2PApplicationPacketHeader *header = reinterpret_cast<P2PApplicationPacketHeader *>(maybe_new_packet->content());
  header->action = action_;
  header->stage = P2PActionStage::kCancel;
  header->request_id = current_request_id_;
  p2p_stream_.output().Commit(maybe_new_packet->priority(), guarantee_delivery_);

  state_ = kIdle;

  OnAbort();
}

P2PActionClient::P2PActionClient(P2PPacketStreamLinux *p2p_stream, const TimerInterface *system_timer, std::mutex *p2p_mutex)
  : p2p_stream_(*ASSERT_NOT_NULL(p2p_stream)), system_timer_(*ASSERT_NOT_NULL(system_timer)), p2p_mutex_(*ASSERT_NOT_NULL(p2p_mutex)) {
  p2p_stream_.other_end_started_callback(P2POtherEndStartedCallback(&P2PActionClient::OnOtherEndStarted, this));
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
  // The caller is responsible for locking p2p_mutex_ before calling this function.

  // Manage lifecycle of all actions by calling Run() on all handlers.
  for (int i = 0; i < sizeof(handlers_) / sizeof(handlers_[0]); ++i) {
    if (handlers_[i] != nullptr) {
      handlers_[i]->Run();
    }
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

  if (handler->state() != P2PActionClientHandlerBase::State::kWaitingForResponse) {
    // The action is not waiting for a response because it was either not started 
    // or it's being cancelled due to the other end having restarted.
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

void P2PActionClient::OnOtherEndStarted(void *p_self) {
  ASSERT_NOT_NULL(p_self);
  P2PActionClient &self = *reinterpret_cast<P2PActionClient *>(p_self);
  // Lock the mutex, so callbacks can alter the handlers' state.
  // std::lock_guard<std::mutex> guard(self.p2p_mutex_);
  for (int i = 0; i < sizeof(handlers_) / sizeof(handlers_[0]); ++i) {
    if (self.handlers_[i] != nullptr) {
      self.handlers_[i]->OnOtherEndStarted();
    }
  }
}

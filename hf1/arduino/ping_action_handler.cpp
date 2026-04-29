#include "ping_action_handler.h"
#include "logger_interface.h"

bool PingActionHandler::OnRequest() {
  // Ping received: notify callback.
  if (ping_received_callback_ != nullptr) {
    ping_received_callback_();
  }
  return P2PActionHandler<P2PPingRequest, P2PPingReply>::OnRequest();
}

bool PingActionHandler::Run() {
  switch(state_) {
    case kProcessingRequest: {
      if (TrySendingReply()) {
        return false; // Reply sent; do not call Run() again.
      }
      state_ = kSendingReply;
      break;
    }
    
    case kSendingReply: {
      if (TrySendingReply()) {
        state_ = kProcessingRequest;
        return false; // Reply sent; do not call Run() again.
      }
      break;
    }
  }
  return true;
}

bool PingActionHandler::TrySendingReply() {
  StatusOr<P2PActionPacketAdapter<P2PPingReply>> maybe_reply = NewReply();
  if (!maybe_reply.ok()) {
    return false;
  }
  P2PActionPacketAdapter<P2PPingReply> reply = *maybe_reply;
  reply.Commit(/*guarantee_delivery=*/true);
  return true;
}

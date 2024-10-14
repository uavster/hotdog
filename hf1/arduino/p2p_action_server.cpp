#include "p2p_action_server.h"
#include "utils.h"
#include "logger_interface.h"

P2PActionServer::P2PActionServer(P2PPacketStreamArduino *p2p_stream)
  : p2p_stream_(*ASSERT_NOT_NULL(p2p_stream)) {
}

P2PActionServer::~P2PActionServer() {
}

void P2PActionServer::Register(P2PActionHandlerBase *handler) {
  ASSERT(!handler->is_registered());
  handlers_[static_cast<int>(handler->action())] = handler;
  handler->is_registered(true);
}

void P2PActionServer::RunActions() {
  // Execute all running actions.
  for (int i = 0; i < P2PAction::kCount; ++i) {
    P2PActionHandlerBase *handler = handlers_[i];
    if (handler == NULL || 
        handler->run_state() == P2PActionHandlerBase::RunState::kIdle) {
      continue;
    }
    if (!handler->Run()) {
      handler->run_state(P2PActionHandlerBase::RunState::kIdle);
    }
  }
}

void P2PActionServer::InitActionsIfNeeded() {
  for (int i = 0; i < P2PAction::kCount; ++i) {
    P2PActionHandlerBase *handler = handlers_[i];
    if (!handler->is_initialized()) {
      handler->Init();
      handler->is_initialized(true);
    }
  }
}

StatusOr<const P2PPacketView> P2PActionServer::GetRequestOrCancellation() const {
  // Check if we got a new action request.
  const auto maybe_oldest_packet_view = p2p_stream_.input().OldestPacket();
  if (!maybe_oldest_packet_view.ok()) {
    // All input packets have been processed.
    return maybe_oldest_packet_view;
  }

  ASSERT(maybe_oldest_packet_view->length() >= sizeof(P2PApplicationPacketHeader));

  const auto app_header = reinterpret_cast<const P2PApplicationPacketHeader *>(maybe_oldest_packet_view->content()); 
  if (app_header->stage != P2PActionStage::kRequest &&
    app_header->stage != P2PActionStage::kCancel) {
    LOG_WARNING("Received packet that is neither a request nor a cancellation.");
    p2p_stream_.input().Consume(maybe_oldest_packet_view->priority());
    return Status::kMalformedError;
  }

  if (app_header->action >= P2PAction::kCount) {
    LOG_WARNING("Received unsupported action.");
    p2p_stream_.input().Consume(maybe_oldest_packet_view->priority());
    return Status::kMalformedError;
  }

  P2PActionHandlerBase *handler = handlers_[app_header->action];
  if (handler == NULL) {
    LOG_ERROR("No handler registered for action.");
    p2p_stream_.input().Consume(maybe_oldest_packet_view->priority());
    return Status::kMalformedError;
  }

  ASSERT(maybe_oldest_packet_view->length() == sizeof(P2PApplicationPacketHeader) + handler->GetExpectedRequestSize());

  return maybe_oldest_packet_view;
}

void P2PActionServer::Run() {
  InitActionsIfNeeded();
  RunActions();

  // Handle action requests and cancellations.
  StatusOr<const P2PPacketView> maybe_packet = GetRequestOrCancellation();
  if (!maybe_packet.ok()) {
    return;
  }
  const auto app_header = reinterpret_cast<const P2PApplicationPacketHeader *>(maybe_packet->content());
  P2PActionHandlerBase *handler = handlers_[app_header->action];
  switch(app_header->stage) {
    case P2PActionStage::kRequest: {
      if (handler->run_state() == P2PActionHandlerBase::RunState::kRunning) {
        LOG_ERROR("Cannot start action when it's already running.");
        break;
      }
      handler->run_state(P2PActionHandlerBase::RunState::kIdle);
      // The handler can first retrieve the request directly from the input stream.
      handler->request_bytes(maybe_packet->content() + sizeof(P2PApplicationPacketHeader));
      // The request determines the action's priority. This affects the reply and progress 
      // priorities, not the action's scheduling.
      handler->request_priority(maybe_packet->priority());
      if (handler->OnRequest()) {
        if (handler->Run()) {
          // The action goes on. Further calls to run will operate on a copy, as the input 
          // packet must be consumed for other packets to be processed.
          memcpy(handler->GetRequestCopyBuffer(), maybe_packet->content() + sizeof(P2PApplicationPacketHeader), handler->GetExpectedRequestSize());
          handler->request_bytes(handler->GetRequestCopyBuffer());    
          handler->run_state(P2PActionHandlerBase::RunState::kRunning);
        }
      }      
      break;
    }
    
    case P2PActionStage::kCancel: {
      if (handler->run_state() != P2PActionHandlerBase::RunState::kRunning) {
        LOG_WARNING("Trying to cancel an action that was not running.");
        break;
      }
      handler->OnCancel();
      handler->run_state(P2PActionHandlerBase::RunState::kIdle);
      break;
    }
  }
  // The action either ended or the packet was copied to the handler, so it's ok to consume
  // it from the input stream for other packets to be processed.
  p2p_stream_.input().Consume(maybe_packet->priority());
}

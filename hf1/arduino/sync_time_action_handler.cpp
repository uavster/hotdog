#include "sync_time_action_handler.h"
#include <kinetis.h>

#define kTimeSyncServerInputPin 8 

void *time_sync_server_singleton = nullptr;

SyncTimeActionHandler::SyncTimeActionHandler(P2PPacketStreamArduino *p2p_stream, TimerInterface *system_timer) 
  : P2PActionHandler<P2PSyncTimeRequest, P2PSyncTimeReply>(P2PAction::kTimeSync, p2p_stream),
    system_timer_(*ASSERT_NOT_NULL(system_timer)) {
  ASSERT(time_sync_server_singleton == nullptr);
  time_sync_server_singleton = this;  
}

void SyncTimeActionHandler::Init() {
  last_edge_detect_local_timestamp_ns_ = -1ULL;
  state_ = WAIT_FOR_TIME_SYNC_REQUEST;
  // Configure rising edge interrupt in pin.
  pinMode(kTimeSyncServerInputPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(kTimeSyncServerInputPin), NotifyEdgeDetected, RISING);
}

void SyncTimeActionHandler::NotifyEdgeDetected() {
  auto *time_sync_server = reinterpret_cast<SyncTimeActionHandler *>(time_sync_server_singleton);
  time_sync_server->last_edge_detect_local_timestamp_ns_ = time_sync_server->system_timer_.GetLocalNanoseconds();
}

bool SyncTimeActionHandler::OnRequest() {
  // Disable the IRQ, as we should not get any other edge on the pin until after
  // sending the reply. This should filter any spurious edges.
  NVIC_DISABLE_IRQ(digitalPinToInterrupt(kTimeSyncServerInputPin));
  
  // By the time we get the request packet, we should have detected the sync signal.
  // Otherwise, we might have started after the other end generated it. In that case,
  // do not reply; the other end will time out and retry.
  if (last_edge_detect_local_timestamp_ns_ == -1ULL) {
    NVIC_ENABLE_IRQ(digitalPinToInterrupt(kTimeSyncServerInputPin));
    return false;
  }

  // The packet should still be in the input stream at this call.
  const auto maybe_request_packet = p2p_stream().input().OldestPacket();
  ASSERT(maybe_request_packet.ok());

  if (maybe_request_packet->reception_local_time_ns() < last_edge_detect_local_timestamp_ns_) {
    // The packet was received before the sync signal was detected; must be a previous
    // sync attempt from the client: reject.
    NVIC_ENABLE_IRQ(digitalPinToInterrupt(kTimeSyncServerInputPin));
    return false;
  }

  // Got a sync signal and a posterior detection timestamp from the other end:
  // update global time offset if necessary.
  const P2PSyncTimeRequest &time_sync_request = GetRequest();
  uint64_t last_edge_detect_remote_timestamp_ns = NetworkToLocal<kP2PLocalEndianness>(time_sync_request.sync_edge_local_timestamp_ns);
  if (last_edge_detect_remote_timestamp_ns > last_edge_detect_local_timestamp_ns_) {
      system_timer_.global_offset_nanoseconds() = last_edge_detect_remote_timestamp_ns - last_edge_detect_local_timestamp_ns_;
      char tmp[32];
      Uint64ToString(system_timer_.global_offset_nanoseconds(), tmp);
      Serial.printf("Global timer +%s ns\n", tmp);
  }
  return true;
}

bool SyncTimeActionHandler::Run() {
  auto maybe_reply = NewReply();
  if (!maybe_reply.ok()) {
    // Output buffer is full: keep trying.
    return true;
  }

  // The edge detection latency on this computer is negligible compared to that on the Linux.
  (*maybe_reply)->sync_edge_local_timestamp_ns = LocalToNetwork<kP2PLocalEndianness>(last_edge_detect_local_timestamp_ns_);

  // No need to guarantee reply delivery: if it is not delivered, the client will just retry the time sync action
  // at a later time, and this packet will not hold back other unguaranteed packets of equal priority or less.
  // This should not be a problem as long as the synchronization action period is low enough to keep 
  // desynchronization at bay despite the previously lost action.
  p2p_stream().output().Commit(request_priority(), /*guarantee_delivery=*/true);

  last_edge_detect_local_timestamp_ns_ = -1ULL;
  // Clear any interrupt request that might not be current and reenable the IRQ,
  // ready to detect a new edge.
  uint32_t isfr = PORTD_ISFR;
  PORTD_ISFR = isfr;
  NVIC_ENABLE_IRQ(digitalPinToInterrupt(kTimeSyncServerInputPin));

  // Synchronization done.
  return false;
}

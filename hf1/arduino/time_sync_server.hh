#include "kinetis.h"
#define kTimeSyncServerInputPin 8 
#define kTimeSyncPacketsPriority P2PPriority::kLow

extern void *time_sync_server_singleton;

#include <p2p_application_protocol.h>

template <int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness>
TimeSyncServer<kInputCapacity, kOutputCapacity, kLocalEndianness>::TimeSyncServer(P2PPacketStream<kInputCapacity, kOutputCapacity, kLocalEndianness> *p2p_packet_stream, TimerInterface *system_timer)
  : p2p_packet_stream_(*p2p_packet_stream), system_timer_(*system_timer) {
  ASSERT(time_sync_server_singleton == nullptr);
  time_sync_server_singleton = this;
}

template <int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness>
void TimeSyncServer<kInputCapacity, kOutputCapacity, kLocalEndianness>::Init() {
  last_edge_detect_local_timestamp_ns_ = -1ULL;
  state_ = WAIT_FOR_TIME_SYNC_REQUEST;
  // Configure rising edge interrupt in pin.
  pinMode(kTimeSyncServerInputPin, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(kTimeSyncServerInputPin), NotifyEdgeDetected, RISING);
}

template <int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness>
void TimeSyncServer<kInputCapacity, kOutputCapacity, kLocalEndianness>::NotifyEdgeDetected() {
  auto *time_sync_server = reinterpret_cast<TimeSyncServer<kInputCapacity, kOutputCapacity, kLocalEndianness> *>(time_sync_server_singleton);
  time_sync_server->last_edge_detect_local_timestamp_ns_ = time_sync_server->system_timer_.GetLocalNanoseconds();
  char tmp[32];
  Uint64ToString(time_sync_server->last_edge_detect_local_timestamp_ns_, tmp);
  Serial.printf("t:%s\n", tmp);
}

template <int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness>
void TimeSyncServer<kInputCapacity, kOutputCapacity, kLocalEndianness>::Run() {
  switch(state_) {
/*    case WAIT_FOR_SYNC_EDGE: {
      // Without this state, 
// Serial.println("WAIT_FOR_SYNC_EDGE");
      const int irq_number = digitalPinToInterrupt(kTimeSyncServerInputPin);
      bool irq_enabled = NVIC_IS_ENABLED(irq_number);
      NVIC_DISABLE_IRQ(irq_number);
      if (last_edge_detect_local_timestamp_ns_ != -1ULL) {
        // Got a sync edge.
Serial.println("Got sync edge");
        state_ = WAIT_FOR_TIME_SYNC_REQUEST;
      }
      if (irq_enabled) { NVIC_ENABLE_IRQ(irq_number); }
      break;
    }
*/
    case WAIT_FOR_TIME_SYNC_REQUEST: {
// Serial.println("WAIT_FOR_TIME_SYNC_REQUEST");
      const auto maybe_oldest_packet_view = p2p_packet_stream_.input().OldestPacket();
      if (!maybe_oldest_packet_view.ok() || 
          maybe_oldest_packet_view->length() < sizeof(P2PApplicationPacketHeader) || 
          reinterpret_cast<const P2PApplicationPacketHeader *>(maybe_oldest_packet_view->content())->command != kP2PCommandTimeSyncRequest) { 
              break;
      }
      ASSERT(maybe_oldest_packet_view->length() == sizeof(P2PApplicationPacketHeader) + sizeof(P2PTimeSyncRequestContent));

      // Disable the IRQ, as we should not get any other edge on the pin until after
      // sending the reply. This should filter any spurious edges.
      NVIC_DISABLE_IRQ(digitalPinToInterrupt(kTimeSyncServerInputPin));
      
      // By the time we get the request packet, we should have detected the sync signal.
      // Otherwise, we might have started after the other end generated it. In that case,
      // do not reply; the other end will time out and retry.
      if (last_edge_detect_local_timestamp_ns_ == -1ULL) {
        p2p_packet_stream_.input().Consume(kTimeSyncPacketsPriority);
        NVIC_ENABLE_IRQ(digitalPinToInterrupt(kTimeSyncServerInputPin));
        break;
      }

char tmp[32];
Uint64ToString(maybe_oldest_packet_view->reception_local_time_ns(), tmp);
Serial.printf("packet_t:%s <? ", tmp);
Uint64ToString(last_edge_detect_local_timestamp_ns_, tmp);
Serial.printf("edge_t:%s\n", tmp);
      if (maybe_oldest_packet_view->reception_local_time_ns() < last_edge_detect_local_timestamp_ns_) {
        // The packet was received before the sync signal was detected; must be a previous
        // sync attempt from the client: reject.
        p2p_packet_stream_.input().Consume(kTimeSyncPacketsPriority);
        NVIC_ENABLE_IRQ(digitalPinToInterrupt(kTimeSyncServerInputPin));
        break;
      }
Serial.printf("len:%d\n", maybe_oldest_packet_view->length());
      
      // Got a sync signal and a posterior detection timestamp from the other end:
      // update global time offset if necessary.
      const auto *time_sync_request = reinterpret_cast<const P2PTimeSyncRequestContent *>(&maybe_oldest_packet_view->content()[sizeof(P2PApplicationPacketHeader)]);
      uint64_t last_edge_detect_remote_timestamp_ns = NetworkToLocal<kLocalEndianness>(time_sync_request->sync_edge_local_timestamp_ns);
      if (last_edge_detect_remote_timestamp_ns > last_edge_detect_local_timestamp_ns_) {
Serial.println("Updated global timer offset.");
          system_timer_.global_offset_nanoseconds() = last_edge_detect_remote_timestamp_ns - last_edge_detect_local_timestamp_ns_;
      }
      p2p_packet_stream_.input().Consume(kTimeSyncPacketsPriority);

      state_ = SEND_TIME_SYNC_REPLY;
      break;
    }

    case SEND_TIME_SYNC_REPLY: {
// Serial.println("SEND_TIME_SYNC_REPLY");
      // Schedule a request with low priority, so that regular time sync does not block urgent communications.
      auto maybe_new_packet = p2p_packet_stream_.output().NewPacket(kTimeSyncPacketsPriority);
      if (!maybe_new_packet.ok()) { break; }

      maybe_new_packet->length() = sizeof(P2PApplicationPacketHeader) + sizeof(P2PTimeSyncReplyContent);
      reinterpret_cast<P2PApplicationPacketHeader *>(maybe_new_packet->content())->command = kP2PCommandTimeSyncReply;
      // The edge detection latency on this computer is negligible compared to that on the Linux.
      reinterpret_cast<P2PTimeSyncReplyContent *>(&maybe_new_packet->content()[sizeof(P2PApplicationPacketHeader)])->sync_edge_local_timestamp_ns = LocalToNetwork<kLocalEndianness>(last_edge_detect_local_timestamp_ns_);
      p2p_packet_stream_.output().Commit(kTimeSyncPacketsPriority, /*guarantee_delivery=*/true);

      last_edge_detect_local_timestamp_ns_ = -1ULL;
      state_ = WAIT_FOR_TIME_SYNC_REQUEST;
      // Clear any interrupt request that might not be current and reenable the IRQ,
      // ready to detect a new edge.
      uint32_t isfr = PORTD_ISFR;
      PORTD_ISFR = isfr;
      NVIC_ENABLE_IRQ(digitalPinToInterrupt(kTimeSyncServerInputPin));
      break;
    }
  }
}
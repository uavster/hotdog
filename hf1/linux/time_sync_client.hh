#include "time_sync_client.h"
#include <p2p_application_protocol.h>
#include <network.h>
#include <JetsonGPIO.h>
#include "logger_interface.h"
#include <sstream>
#include <iostream>

#define kTimeSyncRequestPinNumber 33
#define kTimeSyncLoopbackPinNumber 31

#define kTimeSyncPacketsPriority P2PPriority::kLow

// This is the time to wait before requesting the time sync for the first time.
// It ensures that the GPIO library has finished initializing everything we need.
#define kWarmupDurationNs 100'000'000ULL

// Maximum time between setting the signal in the output pin and capturing the time of the rising edge in the loopback pin.
// The client will repeat the synchronization signal until this constraint is met. The lower the resulting time difference,
// the less uncertainty in the actual time at which the rising edge happened, but the more tries the process might take until
// getting a valid value. If this constraint is too low, the system might not be able to achieve it, and the client will keep
// trying ad infinitum.
#define kMaxSetDetectDurationNs 40'000ULL

// Minimum time interval between consecutive sync signals. Consecutive signals will be generated while trying to meet
// the kMaxSetDetectDurationNs constraint. 
#define kMinTimeBetweenSyncEdgesNs 1'000'000ULL

// Maximum number of attempts at detecting the sync signal in the loopback pin before declaring
// failure.
#define kMaxNumSyncEdgeRetriesBeforFailure 50

// Maximum time to wait for a slot to be available to send the sync request packet. If the wait times
// out, we assume the buffer is saturated because something went wrong and fail.
#define kMaxTimeSyncRequestDelayNs 250'000'000ULL

// Maximum time to wait for a sync packet reply from the other end. If the wait times out, we assume the other end missed the
// sync signal and we generate it again.
#define kMaxTimeSyncReplyDelayNs 250'000'000ULL

extern void *time_sync_client_singleton;

template <int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness>
TimeSyncClient<kInputCapacity, kOutputCapacity, kLocalEndianness>::TimeSyncClient(P2PPacketStream<kInputCapacity, kOutputCapacity, kLocalEndianness> *p2p_packet_stream, TimerInterface *system_timer)
    : p2p_packet_stream_(*p2p_packet_stream), system_timer_(*system_timer), state_(kWarmup), sync_requested_(false), last_sync_status_(SyncStatus::kNotAttempted), last_sync_offset_ns_(0) {
    
    ASSERT(time_sync_client_singleton == nullptr);
    time_sync_client_singleton = this;
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(kTimeSyncRequestPinNumber, GPIO::OUT, GPIO::LOW);
    GPIO::setup(kTimeSyncLoopbackPinNumber, GPIO::IN);
    GPIO::add_event_detect(kTimeSyncLoopbackPinNumber, GPIO::Edge::RISING, EdgeLoopbackCallback);
    creation_time_ = system_timer->GetLocalNanoseconds();
}

template <int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness>
void TimeSyncClient<kInputCapacity, kOutputCapacity, kLocalEndianness>::EdgeLoopbackCallback() {
    auto *time_sync_client = reinterpret_cast<TimeSyncClient<kInputCapacity, kOutputCapacity, kLocalEndianness> *>(time_sync_client_singleton);
    std::lock_guard<std::mutex> guard(time_sync_client->mutex_);
    time_sync_client->last_edge_detect_local_timestamp_ns_ = time_sync_client->system_timer_.GetLocalNanoseconds();
}

template <int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness>
TimeSyncClient<kInputCapacity, kOutputCapacity, kLocalEndianness>::~TimeSyncClient() {
    GPIO::remove_event_detect(kTimeSyncLoopbackPinNumber);
    GPIO::cleanup({ kTimeSyncRequestPinNumber, kTimeSyncLoopbackPinNumber});    
}

template <int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness>
void TimeSyncClient<kInputCapacity, kOutputCapacity, kLocalEndianness>::Run() {
    switch(state_) {
        case kWarmup:
            if (system_timer_.GetLocalNanoseconds() - creation_time_ < kWarmupDurationNs) { break; }

            state_ = kIdle;
            break;

        case kIdle: {
            // Clean input stream from previous responses from the time sync server.
            while(true) {
              const auto maybe_oldest_packet_view = p2p_packet_stream_.input().OldestPacket();
              if (!maybe_oldest_packet_view.ok() ||
                  maybe_oldest_packet_view->length() < sizeof(P2PApplicationPacketHeader) ||
                  reinterpret_cast<const P2PApplicationPacketHeader *>(maybe_oldest_packet_view->content())->action != P2PAction::kTimeSync ||
                  reinterpret_cast<const P2PApplicationPacketHeader *>(maybe_oldest_packet_view->content())->stage != P2PActionStage::kReply) { 
                break;
              }
              ASSERT(maybe_oldest_packet_view->length() == sizeof(P2PApplicationPacketHeader) + sizeof(P2PSyncTimeReply));
              p2p_packet_stream_.input().Consume(maybe_oldest_packet_view->priority());
            }

            // Start synchronization if requested.
            if (sync_requested_) {
                num_sync_attempts_ = 0;
                state_ = kGenerateSyncEdgeAndWaitForLoopback;
                last_sync_status_ = SyncStatus::kInProgress;
            }
            break;
        }

        case kGenerateSyncEdgeAndWaitForLoopback: {
            // Reset request signal.
            GPIO::output(kTimeSyncRequestPinNumber, GPIO::LOW);

            if ((++num_sync_attempts_) > kMaxNumSyncEdgeRetriesBeforFailure) {
              // Too many times: declare failure.
              sync_requested_ = false;
              state_ = kIdle;
              last_sync_status_ = SyncStatus::kError;
              LOG_ERROR("Too many failed synchronization attempts.");
              break;
            }

            {
              // Unlock ASAP to reduce thread contention.
              std::lock_guard<std::mutex> guard(mutex_);
              last_edge_detect_local_timestamp_ns_ = -1ULL;
            }
            // Create a rising edge in the time sync signal.
            last_edge_set_local_timestamp_ns_ = system_timer_.GetLocalNanoseconds();
            GPIO::output(kTimeSyncRequestPinNumber, GPIO::HIGH);

            // Wait for the local edge to be detected locally in the loopback pin.
            // We want low latency, so don't yield time to the caller yet.
            last_edge_detect_local_timestamp_ns_copy_ = -1ULL;
            do {
              // Do we still have time to detect the edge soon enough?
              if (system_timer_.GetLocalNanoseconds() - last_edge_set_local_timestamp_ns_ > kMaxSetDetectDurationNs) {
                  // Too late for a quality edge: retry.
                  last_edge_attempt_timestamp_ns_ = system_timer_.GetLocalNanoseconds();
                  state_ = kWaitToRegenerateSyncEdge;
                  break;
              }
              // There's still time: check if we got the edge.
              {
                std::lock_guard<std::mutex> guard(mutex_);
                last_edge_detect_local_timestamp_ns_copy_ = last_edge_detect_local_timestamp_ns_;
              }
            } while(last_edge_detect_local_timestamp_ns_copy_ == -1ULL);
            if (state_ != kGenerateSyncEdgeAndWaitForLoopback) {
              // Something went wrong: move to the new state.
              break;
            }
            // The rising edge has been detected soon enough for good synchronization.
            state_ = kSendTimeSyncRequest;
            // Reset request pin.
            GPIO::output(kTimeSyncRequestPinNumber, GPIO::LOW);
            break;
        }

        case kWaitToRegenerateSyncEdge:
            if (system_timer_.GetLocalNanoseconds() - last_edge_attempt_timestamp_ns_ < kMinTimeBetweenSyncEdgesNs) { break; }

            state_ = kGenerateSyncEdgeAndWaitForLoopback;
            break;

        case kSendTimeSyncRequest: {
            // Schedule a request with low priority, so that regular time sync does not block urgent communications.
            auto maybe_new_packet = p2p_packet_stream_.output().NewPacket(kTimeSyncPacketsPriority);
            if (!maybe_new_packet.ok()) {
              if (system_timer_.GetLocalNanoseconds() - last_edge_detect_local_timestamp_ns_copy_ > kMaxTimeSyncRequestDelayNs) {
                  // The output buffer is saturated for too long: fail now and let the caller retry at
                  // a later time.
                  sync_requested_ = false;
                  state_ = kIdle;
                  last_sync_status_ = SyncStatus::kError;
                  LOG_ERROR("P2P output queue is saturated.");
                  break;
              } else {
                  // Try again for an available slot.
                  break;
              }
            }

            // It is guaranteed that the rising edge will have been processed in the other end by the time the request is received.
            maybe_new_packet->length() = sizeof(P2PApplicationPacketHeader) + sizeof(P2PSyncTimeRequest);
            reinterpret_cast<P2PApplicationPacketHeader *>(maybe_new_packet->content())->action = P2PAction::kTimeSync;
            reinterpret_cast<P2PApplicationPacketHeader *>(maybe_new_packet->content())->stage = P2PActionStage::kRequest;
            // The edge was received some time between setting the output pin and receiving the event from the loopback pin: use the mid-point.
            last_edge_estimated_local_timestamp_ns_ = (last_edge_set_local_timestamp_ns_ + last_edge_detect_local_timestamp_ns_copy_) / 2;
            reinterpret_cast<P2PSyncTimeRequest *>(&maybe_new_packet->content()[sizeof(P2PApplicationPacketHeader)])->sync_edge_local_timestamp_ns = LocalToNetwork<kLocalEndianness>(last_edge_estimated_local_timestamp_ns_);
            p2p_packet_stream_.output().Commit(kTimeSyncPacketsPriority, /*guarantee_delivery=*/false);
            request_sent_timestamp_ns_ = system_timer_.GetLocalNanoseconds();
            state_ = kWaitForTimeSyncReply;
            break;
        }
        
        case kWaitForTimeSyncReply: {
            if (system_timer_.GetLocalNanoseconds() - request_sent_timestamp_ns_ > kMaxTimeSyncReplyDelayNs) { 
                // The other end must have missed the sync signal (e.g. it started after this end): start over.                
                state_ = kGenerateSyncEdgeAndWaitForLoopback;
                break;
            }

            // The other end should reply with the timestamp at which it detected the rising edge.
            const auto maybe_oldest_packet_view = p2p_packet_stream_.input().OldestPacket();
            if (!maybe_oldest_packet_view.ok() || 
                maybe_oldest_packet_view->length() < sizeof(P2PApplicationPacketHeader) || 
                reinterpret_cast<const P2PApplicationPacketHeader *>(maybe_oldest_packet_view->content())->action != P2PAction::kTimeSync ||
                reinterpret_cast<const P2PApplicationPacketHeader *>(maybe_oldest_packet_view->content())->stage != P2PActionStage::kReply) { 
                break;
            }

            ASSERT(maybe_oldest_packet_view->length() == sizeof(P2PApplicationPacketHeader) + sizeof(P2PSyncTimeReply));
            const auto *time_sync_reply = reinterpret_cast<const P2PSyncTimeReply *>(&maybe_oldest_packet_view->content()[sizeof(P2PApplicationPacketHeader)]);
            const auto sync_edge_remote_timestamp_ns = NetworkToLocal<kLocalEndianness>(time_sync_reply->sync_edge_local_timestamp_ns);
            if (sync_edge_remote_timestamp_ns >= last_edge_estimated_local_timestamp_ns_) {
                // Only advance the clock that is behind, so that all clocks stay monotonic.
                last_sync_offset_ns_ = sync_edge_remote_timestamp_ns - last_edge_estimated_local_timestamp_ns_;
                system_timer_.global_offset_nanoseconds() = last_sync_offset_ns_;
            } else {
              last_sync_offset_ns_ = -(last_edge_estimated_local_timestamp_ns_ - sync_edge_remote_timestamp_ns);
            }
            p2p_packet_stream_.input().Consume(maybe_oldest_packet_view->priority());

            sync_requested_ = false;
            state_ = kIdle;
            last_sync_status_ = SyncStatus::kOk;
            break;
        }
    }
}

template <int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness>
void TimeSyncClient<kInputCapacity, kOutputCapacity, kLocalEndianness>::RequestTimeSync() {
    sync_requested_ = true;
}

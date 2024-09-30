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
#define kWarmupDurationNs 100000000ULL

// Maximum time between setting the signal in the output pin and capturing the time of the rising edge in the loopback pin.
// The client will repeat the synchronization signal until this constraint is met. The lower the resulting time difference,
// the less uncertainty in the actual time at which the rising edge happened, but the more tries the process might take until
// getting a valid value. If this constraint is too low, the system might not be able to achieve it, and the client will keep
// trying ad infinitum.
#define kMaxSetDetectDurationNs 40000ULL

// Minimum time interval between consecutive sync signals. Consecutive signals will be generated while trying to meet
// the kMaxSetDetectDurationNs constraint. 
#define kMinTimeBetweenSyncEdges 1000000ULL

// Maximum time to wait for a sync packet reply from the other end. If the wait times out, we assume the other end missed the
// sync signal and we generate it again.
#define kMaxTimeSyncReplyDelayNs 250000000ULL

extern void *time_sync_client_singleton;

template <int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness>
TimeSyncClient<kInputCapacity, kOutputCapacity, kLocalEndianness>::TimeSyncClient(P2PPacketStream<kInputCapacity, kOutputCapacity, kLocalEndianness> *p2p_packet_stream, TimerInterface *system_timer)
    : p2p_packet_stream_(*p2p_packet_stream), system_timer_(*system_timer), state_(WARMUP), sync_requested_(false) {
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
    GPIO::cleanup();    
}

template <int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness>
void TimeSyncClient<kInputCapacity, kOutputCapacity, kLocalEndianness>::Run() {
    switch(state_) {
        case WARMUP:
        std::cout << "a" << std::endl;
            if (system_timer_.GetLocalNanoseconds() - creation_time_ < kWarmupDurationNs) { break; }

            state_ = IDLE;
            break;

        case IDLE:
        std::cout << "b" << std::endl;
            if (sync_requested_) {
                state_ = GENERATE_SYNC_EDGE;
            }
            break;

        case GENERATE_SYNC_EDGE: {
        std::cout << "c" << std::endl;
            // Create a rising edge in the time sync signal.
            last_edge_set_local_timestamp_ns_ = system_timer_.GetLocalNanoseconds();
            mutex_.lock();
            last_edge_detect_local_timestamp_ns_ = -1ULL;
            mutex_.unlock();    // Unlock ASAP to reduce thread contention.
            GPIO::output(kTimeSyncRequestPinNumber, GPIO::HIGH);
            state_ = WAIT_FOR_LOCAL_EDGE;
            break;
        }

        case WAIT_FOR_LOCAL_EDGE: {
        std::cout << "d" << std::endl;
            std::lock_guard<std::mutex> guard(mutex_);
            if (last_edge_detect_local_timestamp_ns_ == -1ULL) { break; }

            GPIO::output(kTimeSyncRequestPinNumber, GPIO::LOW);

            uint64_t diff_ns = last_edge_detect_local_timestamp_ns_ - last_edge_set_local_timestamp_ns_;
            if (diff_ns > kMaxSetDetectDurationNs) {
                last_edge_attempt_timestamp_ns_ = system_timer_.GetLocalNanoseconds();
                state_ = WAIT_TO_REGENERATE_SYNC_EDGE;
                break;
            }

            // The rising edge was detected in the loopback pin.
            state_ = SEND_TIME_SYNC_REQUEST;
            break;
        }

        case WAIT_TO_REGENERATE_SYNC_EDGE:
        std::cout << "e" << std::endl;
            if (system_timer_.GetLocalNanoseconds() - last_edge_attempt_timestamp_ns_ < kMinTimeBetweenSyncEdges) { break; }

            state_ = GENERATE_SYNC_EDGE;
            break;

        case SEND_TIME_SYNC_REQUEST: {
        std::cout << "f" << std::endl;
            // Schedule a request with low priority, so that regular time sync does not block urgent communications.
            auto maybe_new_packet = p2p_packet_stream_.output().NewPacket(kTimeSyncPacketsPriority);
            if (!maybe_new_packet.ok()) { break; }

            // It is guaranteed that the rising edge will have been processed in the other end by the time the request is received.
            maybe_new_packet->length() = sizeof(P2PApplicationPacketHeader) + sizeof(P2PTimeSyncRequestContent);
            reinterpret_cast<P2PApplicationPacketHeader *>(maybe_new_packet->content())->command = kP2PCommandTimeSyncRequest;
            // The edge was received some time between setting the output pin and receiving the event from the loopback pin: use the mid-point.
            last_edge_estimated_local_timestamp_ns_ = (last_edge_set_local_timestamp_ns_ + last_edge_detect_local_timestamp_ns_) / 2;
            reinterpret_cast<P2PTimeSyncRequestContent *>(&maybe_new_packet->content()[sizeof(P2PApplicationPacketHeader)])->sync_edge_local_timestamp_ns = LocalToNetwork<kLocalEndianness>(last_edge_estimated_local_timestamp_ns_);
            p2p_packet_stream_.output().Commit(kTimeSyncPacketsPriority, /*guarantee_delivery=*/true);
            request_sent_timestamp_ns_ = system_timer_.GetLocalNanoseconds();
            state_ = WAIT_FOR_TIME_SYNC_REPLY;
            break;
        }
        
        case WAIT_FOR_TIME_SYNC_REPLY: {
        std::cout << "g" << std::endl;
            if (system_timer_.GetLocalNanoseconds() - request_sent_timestamp_ns_ > kMaxTimeSyncReplyDelayNs) { 
                // The other end must have missed the sync signal (e.g. it started after this end): start over.
                state_ = GENERATE_SYNC_EDGE;
                break;
            }

            // The other end should reply with the timestamp at which it detected the rising edge.
            const auto maybe_oldest_packet_view = p2p_packet_stream_.input().OldestPacket();
            if (!maybe_oldest_packet_view.ok() || 
                maybe_oldest_packet_view->length() < sizeof(P2PApplicationPacketHeader) || 
                reinterpret_cast<const P2PApplicationPacketHeader *>(maybe_oldest_packet_view->content())->command != kP2PCommandTimeSyncReply) { 
                    break;
            }

            ASSERT(maybe_oldest_packet_view->length() == sizeof(P2PApplicationPacketHeader) + sizeof(P2PTimeSyncReplyContent));
            const auto *time_sync_reply = reinterpret_cast<const P2PTimeSyncReplyContent *>(&maybe_oldest_packet_view->content()[sizeof(P2PApplicationPacketHeader)]);
            if (time_sync_reply->sync_edge_local_timestamp_ns > last_edge_estimated_local_timestamp_ns_) {
                system_timer_.global_offset_nanoseconds() = NetworkToLocal<kLocalEndianness>(time_sync_reply->sync_edge_local_timestamp_ns) - last_edge_estimated_local_timestamp_ns_;
                std::ostringstream oss;
                oss << "Global timer +" << system_timer_.global_offset_nanoseconds() << " ns";
                LOG_INFO(oss.str().c_str());
            }
            p2p_packet_stream_.input().Consume(kTimeSyncPacketsPriority);

            sync_requested_ = false;
            state_ = IDLE;
            break;
        }
    }
}

template <int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness>
void TimeSyncClient<kInputCapacity, kOutputCapacity, kLocalEndianness>::RequestTimeSync() {
    sync_requested_ = true;
}

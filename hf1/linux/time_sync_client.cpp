#include "time_sync_client.h"
#include <p2p_application_protocol.h>
#include <network.h>
#include <JetsonGPIO.h>
#include "logger_interface.h"
#include <sstream>
#include <iostream>

#define kTimeSyncRequestPinNumber 33
#define kTimeSyncLoopbackPinNumber 31

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

static void *time_sync_client_singleton;

TimeSyncClient::TimeSyncClient(SyncTimeActionClientHandler *sync_action_handler, TimerInterface *system_timer)
    : sync_action_handler_(*ASSERT_NOT_NULL(sync_action_handler)), 
      system_timer_(*ASSERT_NOT_NULL(system_timer)), 
      state_(kWarmup), sync_requested_(false), 
      last_sync_status_(SyncStatus::kNotAttempted), 
      last_sync_offset_ns_(0) {
    
    ASSERT(time_sync_client_singleton == nullptr);
    time_sync_client_singleton = this;
    // Tell the time sync action handler to notify this client with replies from the motion
    // controller.
    sync_action_handler_.AssociateTimeSyncClient(this);
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(kTimeSyncRequestPinNumber, GPIO::OUT, GPIO::LOW);
    GPIO::setup(kTimeSyncLoopbackPinNumber, GPIO::IN);
    GPIO::add_event_detect(kTimeSyncLoopbackPinNumber, GPIO::Edge::RISING, EdgeLoopbackCallback);
    creation_time_ = system_timer->GetLocalNanoseconds();
}

void TimeSyncClient::EdgeLoopbackCallback() {
    auto *time_sync_client = reinterpret_cast<TimeSyncClient *>(time_sync_client_singleton);
    std::lock_guard<std::mutex> guard(time_sync_client->mutex_);
    time_sync_client->last_edge_detect_local_timestamp_ns_ = time_sync_client->system_timer_.GetLocalNanoseconds();
}

TimeSyncClient::~TimeSyncClient() {
    GPIO::remove_event_detect(kTimeSyncLoopbackPinNumber);
    GPIO::cleanup({ kTimeSyncRequestPinNumber, kTimeSyncLoopbackPinNumber});    
}

void TimeSyncClient::ResetReply() {
    std::lock_guard<std::mutex> guard(reply_mutex_);
    reply_ = std::nullopt;
}

void TimeSyncClient::OnReply(const P2PSyncTimeReply &reply) {
    std::lock_guard<std::mutex> guard(reply_mutex_);
    reply_ = reply;
}

std::optional<P2PSyncTimeReply> TimeSyncClient::reply() {
    std::lock_guard<std::mutex> guard(reply_mutex_);
    return reply_;
}

void TimeSyncClient::Run() {
    switch(state_) {
        case kWarmup:
            if (system_timer_.GetLocalNanoseconds() - creation_time_ < kWarmupDurationNs) { break; }

            state_ = kIdle;
            break;

        case kIdle: {
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
            P2PSyncTimeRequest request;
            // The edge was received some time between setting the output pin and receiving the event from the loopback pin: use the mid-point.
            // It is guaranteed that the rising edge will have been processed in the other end by the time the request is received.
            last_edge_estimated_local_timestamp_ns_ = (last_edge_set_local_timestamp_ns_ + last_edge_detect_local_timestamp_ns_copy_) / 2;
            request.sync_edge_local_timestamp_ns = LocalToNetwork<kP2PLocalEndianness>(last_edge_estimated_local_timestamp_ns_);
            if (!sync_action_handler_.Request(request)) {
              if (system_timer_.GetLocalNanoseconds() - last_edge_detect_local_timestamp_ns_copy_ > kMaxTimeSyncRequestDelayNs) {
                  // The output buffer is saturated for too long: fail now and let the caller retry at
                  // a later time.
                  sync_requested_ = false;
                  state_ = kIdle;
                  last_sync_status_ = SyncStatus::kError;
                  LOG_ERROR("P2P output queue is saturated.");
              }
              // Try again for an available slot.
              break;
            }

            request_sent_timestamp_ns_ = system_timer_.GetLocalNanoseconds();
            ResetReply();
            state_ = kWaitForTimeSyncReply;
            break;
        }
        
        case kWaitForTimeSyncReply: {
            if (system_timer_.GetLocalNanoseconds() - request_sent_timestamp_ns_ > kMaxTimeSyncReplyDelayNs) { 
                // The other end must have missed the sync signal (e.g. it started after this end): start over.                
                state_ = kGenerateSyncEdgeAndWaitForLoopback;
                // Cancel the action before retrying.
                sync_action_handler_.Cancel();
                break;
            }

            // The other end should reply with the timestamp at which it detected the rising edge.
            std::optional<P2PSyncTimeReply> maybe_reply = reply();
            if (!reply_.has_value()) {
              break;
            }

            const auto sync_edge_remote_timestamp_ns = NetworkToLocal<kP2PLocalEndianness>(maybe_reply->sync_edge_local_timestamp_ns);
            if (sync_edge_remote_timestamp_ns >= last_edge_estimated_local_timestamp_ns_) {
                // Only advance the clock that is behind, so that all clocks stay monotonic.
                last_sync_offset_ns_ = sync_edge_remote_timestamp_ns - last_edge_estimated_local_timestamp_ns_;
                system_timer_.global_offset_nanoseconds() = last_sync_offset_ns_;
            } else {
              last_sync_offset_ns_ = -(last_edge_estimated_local_timestamp_ns_ - sync_edge_remote_timestamp_ns);
            }

            sync_requested_ = false;
            state_ = kIdle;
            last_sync_status_ = SyncStatus::kOk;
            break;
        }
    }
}

void TimeSyncClient::RequestTimeSync() {
    sync_requested_ = true;
}

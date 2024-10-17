#ifndef TIME_SYNC_CLIENT_INCLUDED_
#define TIME_SYNC_CLIENT_INCLUDED_

#include "p2p_action_client.h"
#include "logger_interface.h"
#include "p2p_packet_stream_linux.h"
#include <network.h>
#include <mutex>
#include <atomic>
#include <optional>

// Packets are low priority and without guaranteed delivery to ease all other traffic.
// If delivery fails, the client will retry.
#define kTimeSyncPacketsDefaultPriority P2PPriority::kLow
#define kTimeSyncPacketsDefaultGuaranteeDelivery false

class SyncTimeActionClientHandler;

// Synchronizes time with the Arduino end when requested.
// It uses a GPIO signal readable by both ends to take a snapshot of both timers, which 
// are then exchanged over the P2P link.
// Only one instance of this class may exist at a time.
class TimeSyncClient {
public:
    friend class SyncTimeActionClientHandler;

    // Does not take ownsership of the pointees, which must outlive this object.
    TimeSyncClient(SyncTimeActionClientHandler *sync_action_handler, TimerInterface *system_timer);
    virtual ~TimeSyncClient();

    // Runs the client logic.
    void Run();

    // Initiates the time synchronization with the low-level computer.
    void RequestTimeSync();

    bool sync_in_progress() const { return sync_requested_; }

    using SyncStatus = enum {
        kNotAttempted,
        kInProgress,
        kOk,
        kError
    };
    SyncStatus last_sync_status() const { return last_sync_status_; }

    // Returns the difference between the local time and the time of the remote end.
    // A positive value is the adjustment added as an offset to the local time to get the global 
    // time.
    // A negative value is the adjustment subctracted as an offset from the remote time to get the
    // global time.
    int64_t last_sync_offset_ns() const { return last_sync_offset_ns_; }

protected:
    // The GPIO library only supports equality-comparable functions, so we need this static accessing the singleton TimeSyncClient.
    static void EdgeLoopbackCallback();
    void ResetReply();
    void OnReply(const P2PSyncTimeReply &reply);
    std::optional<P2PSyncTimeReply> reply();

private:
    SyncTimeActionClientHandler &sync_action_handler_;
    TimerInterface &system_timer_;
    uint64_t creation_time_;
    enum { kWarmup, kIdle, kGenerateSyncEdgeAndWaitForLoopback, kWaitToRegenerateSyncEdge, kSendTimeSyncRequest, kWaitForTimeSyncReply } state_;    
    std::mutex mutex_;
    std::atomic<bool> sync_requested_;
    uint64_t last_edge_set_local_timestamp_ns_;
    uint64_t last_edge_detect_local_timestamp_ns_;
    uint64_t last_edge_detect_local_timestamp_ns_copy_;
    int num_sync_attempts_;
    uint64_t last_edge_estimated_local_timestamp_ns_;
    uint64_t last_edge_attempt_timestamp_ns_;
    uint64_t request_sent_timestamp_ns_;

    SyncStatus last_sync_status_;
    int64_t last_sync_offset_ns_;

    std::mutex reply_mutex_;
    std::optional<P2PSyncTimeReply> reply_;
};

// Provides a mechanism for the TimeSyncClient to send time sync requests to the motion
// controller and receive replies over the P2P link.
class SyncTimeActionClientHandler : public P2PActionClientHandler<P2PSyncTimeRequest, P2PSyncTimeReply, P2PVoid> {
public:
  // Does not take ownsership of the pointees, which must outlive this object.
  SyncTimeActionClientHandler(P2PAction action, P2PPacketStreamLinux *p2p_stream, std::mutex *p2p_mutex) :
    P2PActionClientHandler<P2PSyncTimeRequest, P2PSyncTimeReply, P2PVoid>(action, kTimeSyncPacketsDefaultPriority, kTimeSyncPacketsDefaultGuaranteeDelivery, p2p_stream, p2p_mutex, /*allows_concurrent_requests=*/false), 
    time_sync_client_(nullptr) {}

  // Does not take ownership of the pointee, which must outlive this object.
  void AssociateTimeSyncClient(TimeSyncClient *time_sync_client) {
    ASSERTM(time_sync_client_ == nullptr, "SyncTimeActionClientHandler can only be associated to one TimeSyncClient.");
    time_sync_client_ = time_sync_client;
  }

  void OnReply(const P2PSyncTimeReply &reply) override {
    ASSERTM(time_sync_client_ != nullptr, "SyncTimeActionClientHandler was not associated to a TimeSyncClient.");
    time_sync_client_->OnReply(reply);
  }

private:
  TimeSyncClient *time_sync_client_;
};

#endif  // TIME_SYNC_CLIENT_INCLUDED_
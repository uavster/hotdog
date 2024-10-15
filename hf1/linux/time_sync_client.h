#ifndef TIME_SYNC_CLIENT_INCLUDED_
#define TIME_SYNC_CLIENT_INCLUDED_

#include <p2p_packet_stream.h>
#include <network.h>
#include <mutex>
#include <atomic>

// All necessary operations for time synchronization. 
template<int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness> class TimeSyncClient {
public:
    // Does not take ownsership of the pointees, which must outlive this object.
    TimeSyncClient(P2PPacketStream<kInputCapacity, kOutputCapacity, kLocalEndianness> *p2p_packet_stream, TimerInterface *system_timer);
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

private:
    static TimeSyncClient<kInputCapacity, kOutputCapacity, kLocalEndianness> *singleton_;
    P2PPacketStream<kInputCapacity, kOutputCapacity, kLocalEndianness> &p2p_packet_stream_;
    TimerInterface &system_timer_;
    uint64_t creation_time_;
    enum { WARMUP, IDLE, GENERATE_SYNC_EDGE_AND_WAIT_FOR_LOOPBACK, WAIT_TO_REGENERATE_SYNC_EDGE, SEND_TIME_SYNC_REQUEST, WAIT_FOR_TIME_SYNC_REPLY } state_;    
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
};

#include "time_sync_client.hh"

#endif  // TIME_SYNC_CLIENT_INCLUDED_
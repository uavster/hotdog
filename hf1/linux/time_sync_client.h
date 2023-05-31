#ifndef TIME_SYNC_CLIENT_INCLUDED_
#define TIME_SYNC_CLIENT_INCLUDED_

#include <p2p_packet_stream.h>
#include <network.h>
#include <mutex>

template<int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness> class TimeSyncClient {
public:
    // Does not take ownsership of the pointees, which must outlive this object.
    TimeSyncClient(P2PPacketStream<kInputCapacity, kOutputCapacity, kLocalEndianness> *p2p_packet_stream, TimerInterface *system_timer);
    virtual ~TimeSyncClient();

    // Runs the client logic. Returns true if a new time sync point is available.
    void Run();

    // Initiates the time synchronization with the low-level computer.
    void RequestTimeSync();

protected:
    // The GPIO library only supports equality-comparable functions, so we need this static accessing the singleton TimeSyncClient.
    static void EdgeLoopbackCallback();

private:
    static TimeSyncClient<kInputCapacity, kOutputCapacity, kLocalEndianness> *singleton_;
    P2PPacketStream<kInputCapacity, kOutputCapacity, kLocalEndianness> &p2p_packet_stream_;
    TimerInterface &system_timer_;
    uint64_t creation_time_;
    enum { WARMUP, IDLE, GENERATE_SYNC_EDGE, WAIT_TO_REGENERATE_SYNC_EDGE, WAIT_FOR_LOCAL_EDGE, SEND_TIME_SYNC_REQUEST, WAIT_FOR_TIME_SYNC_REPLY } state_;    
    std::mutex mutex_;
    bool sync_requested_;
    uint64_t last_edge_set_local_timestamp_ns_;
    uint64_t last_edge_detect_local_timestamp_ns_;
    uint64_t last_edge_estimated_local_timestamp_ns_;
    uint64_t last_edge_attempt_timestamp_ns_;
};

#include "time_sync_client.hh"

#endif  // TIME_SYNC_CLIENT_INCLUDED_
#ifndef TIME_SYNC_SERVER_INCLUDED_
#define TIME_SYNC_SERVER_INCLUDED_

#include "logger.h"
#include <p2p_packet_stream.h>
#include "timer.h"

template<int kInputCapacity, int kOutputCapacity, Endianness kLocalEndianness> class TimeSyncServer {
public:
  // Does not take ownsership of the pointees, which must outlive this object.
  TimeSyncServer(P2PPacketStream<kInputCapacity, kOutputCapacity, kLocalEndianness> *p2p_packet_stream, TimerInterface *system_timer);

  // Initializes the object. Call from setup().
  void Init();
  
  // Runs the server logic.
  void Run();

  // Call when the rising edge in the input pin is detected.
  static void NotifyEdgeDetected();

private:
  P2PPacketStream<kInputCapacity, kOutputCapacity, kLocalEndianness> &p2p_packet_stream_;
  TimerInterface &system_timer_;
  enum { WAIT_FOR_TIME_SYNC_REQUEST, SEND_TIME_SYNC_REPLY } state_;
  uint64_t last_edge_detect_local_timestamp_ns_;
};

#include "time_sync_server.hh"

#endif  // TIME_SYNC_SERVER_INCLUDED_

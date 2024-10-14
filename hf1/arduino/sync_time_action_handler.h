#ifndef SYNC_TIME_ACTION_HANDLER_
#define SYNC_TIME_ACTION_HANDLER_

#include "p2p_action_server.h"

class SyncTimeActionHandler : public P2PActionHandler<P2PSyncTimeRequest, P2PSyncTimeReply> {
public:
  // Does not take ownsership of the pointees, which must outlive this object.
  SyncTimeActionHandler(P2PPacketStreamArduino *p2p_stream, TimerInterface *system_timer);

  bool OnRequest() override;
  void Init() override;
  bool Run() override;

  // Call when the rising edge in the input pin is detected.
  static void NotifyEdgeDetected();

private:
  TimerInterface &system_timer_;
  enum { WAIT_FOR_TIME_SYNC_REQUEST, SEND_TIME_SYNC_REPLY } state_;
  uint64_t last_edge_detect_local_timestamp_ns_;
};

#endif  // SYNC_TIME_ACTION_HANDLER_
#ifndef MONITOR_BASE_STATE_ACTION_HANDLER_
#define MONITOR_BASE_STATE_ACTION_HANDLER_

#include "p2p_action_server.h"
#include "timer_interface.h"

class MonitorBaseStateActionHandler : public P2PActionHandler<P2PMonitorBaseStateRequest, P2PMonitorBaseStateReply, P2PMonitorBaseStateProgress> {
public:
  // Does not take ownsership of the pointee, which must outlive this object.
  MonitorBaseStateActionHandler(P2PPacketStreamArduino *p2p_stream, TimerInterface *system_timer)
    : P2PActionHandler<P2PMonitorBaseStateRequest, P2PMonitorBaseStateReply, P2PMonitorBaseStateProgress>(P2PAction::kMonitorBaseState, p2p_stream), 
      system_timer_(*ASSERT_NOT_NULL(system_timer)) {}

  bool Run() override;

private:
  bool TrySendingReply();
  bool TrySendingProgress();

  TimerInterface &system_timer_;
  int num_remaining_messages_;
  uint64_t last_state_update_ns_;
  enum { kReceiveRequest, kWaitForNextBaseState, kSendingReply, kSendingProgress } state_ = kReceiveRequest;
};

#endif  // MONITOR_BASE_STATE_ACTION_HANDLER_
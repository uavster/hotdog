#ifndef P2P_ACTION_CLIENT_STATUS_
#define P2P_ACTION_CLIENT_STATUS_

#include "periodic_runnable.h"

// Keeps track of the status of the action client at the other end of the P2P link.
// When a ping action is received from the client, the status is kept as kActive.
// When no ping action is received from the client for some time, the status turns kInactive.
class P2PActionClientStatus : public PeriodicRunnable {
public:
  enum LinkStatus { kInactive, kActive };

  P2PActionClientStatus(const char *name);

  // Must be called whenever a ping action is received.
  void NotifyPingReceived();

  LinkStatus link_status() const { return link_status_; }

protected:
  void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override;

private:
  LinkStatus link_status_; 
  TimerNanosType last_ping_notification_ns_;
};

#endif  // P2P_ACTION_CLIENT_STATUS_
#ifndef P2P_ACTION_CLIENT_STATUS_
#define P2P_ACTION_CLIENT_STATUS_

#include "periodic_runnable.h"

// Keeps track of the status of the action client at the other end of the P2P link.
// When a ping action is received from the client, the status is kept as kActive.
// When no ping action is received from the client for some time, the status turns kInactive.
class P2PActionClientStatus : public PeriodicRunnable {
public:
  enum LinkStatus { kInactive, kActive };
  using LinkStatusChangedCallback = void (*)(LinkStatus old_status, LinkStatus new_status);

  // Creates the object with a `name`.
  // `link_status_changed_callback`, if not nulptr, is called every time the link status changes.
  // Copies `name` to an internal buffer, so it can be discarded constructing the object.
  // Takes ownership of `link_status_changed_callback`, which cannot be rerenced by the caller after this call.
  P2PActionClientStatus(const char *name, LinkStatusChangedCallback link_status_changed_callback);

  // Must be called whenever a ping action is received.
  void NotifyPingReceived();

  LinkStatus link_status() const { return link_status_; }

protected:
  void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override;

private:
  LinkStatus link_status_; 
  TimerNanosType last_ping_notification_ns_;
  LinkStatusChangedCallback link_status_changed_callback_;
};

#endif  // P2P_ACTION_CLIENT_STATUS_
#include "p2p_action_client_status.h"
#include "logger_interface.h"

constexpr TimerNanosType kPeriodWithoutPingForInactiveNs = 4'000'000'000ULL;
constexpr TimerNanosType kRunPeriodNs = 1'000'000'000ULL;

P2PActionClientStatus::P2PActionClientStatus(const char *name)
  : PeriodicRunnable(name, kRunPeriodNs), link_status_(kInactive), last_ping_notification_ns_(-1ULL) {}

void P2PActionClientStatus::NotifyPingReceived() {
  last_ping_notification_ns_ = GetTimerNanoseconds();
  if (link_status_ == LinkStatus::kInactive) {
    LOG_INFO("P2P action client is now active.");
  }
  link_status_ = LinkStatus::kActive;
}

void P2PActionClientStatus::RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) {
  if (last_ping_notification_ns_ != -1ULL && now_nanos - last_ping_notification_ns_ >= kPeriodWithoutPingForInactiveNs) {
    if (link_status_ == LinkStatus::kActive) {
      LOG_INFO("P2P action client is now inactive.");
    }
    link_status_ = LinkStatus::kInactive;
  }
}
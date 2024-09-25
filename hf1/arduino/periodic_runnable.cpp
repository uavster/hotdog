#include "periodic_runnable.h"

PeriodicRunnable::PeriodicRunnable(TimerNanosType period_nanos)
  : is_first_run_(true), period_nanos_(period_nanos), last_call_nanos_(0) {}

PeriodicRunnable::PeriodicRunnable(TimerSecondsType period_seconds)
  : PeriodicRunnable(NanosFromSeconds(period_seconds)) {}

void PeriodicRunnable::Run() {
  const auto now_nanos = GetTimerNanoseconds();
  if (is_first_run_) {
    is_first_run_ = false;
    last_call_nanos_ = now_nanos;
    RunFirstTime(now_nanos);
    return;
  }

  const TimerNanosType nanos_since_last_call = now_nanos - last_call_nanos_;
  if (nanos_since_last_call < period_nanos_) {
    // Period not elapsed yet.
    return;
  }
  last_call_nanos_ = now_nanos;

  // Period elapsed.
  RunAfterPeriod(now_nanos, nanos_since_last_call);
}
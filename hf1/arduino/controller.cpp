#include "controller.h"

Controller::Controller(const char *name, float run_period_seconds) 
  : PeriodicRunnable(name, run_period_seconds) {}

void Controller::RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) {
  Update(SecondsFromNanos(now_nanos));
}


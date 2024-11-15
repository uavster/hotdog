#include "controller.h"

Controller::Controller(float run_period_seconds) 
  : PeriodicRunnable(run_period_seconds) {}

void Controller::RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) {
  Update(SecondsFromNanos(now_nanos));
}


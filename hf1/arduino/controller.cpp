#include "controller.h"

Controller::Controller(const char *name, float run_period_seconds) 
  : PeriodicRunnable(name, run_period_seconds), is_started_(false) {}

void Controller::RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) {
  if (!is_started_) {
    return;
  }
  Update(SecondsFromNanos(now_nanos) - start_seconds_);
}

void Controller::Start() {
  is_started_ = true;
  start_seconds_ = GetTimerSeconds();
}

void Controller::Stop() {
  StopControl();
  is_started_ = false;
}


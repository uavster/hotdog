#include "periodic_runnable.h"
#if kEnableStats
#include "logger.h"
#include <cstring>
#include <stdio.h>
#endif
#define kPrintStatsPeriodNs 1'000'000'000

PeriodicRunnable::PeriodicRunnable(const char *name, TimerNanosType period_nanos)
  : is_first_run_(true), 
    period_nanos_(period_nanos), 
  #if kEnableStats
    last_call_nanos_(0), 
    last_num_runs_(0), 
    last_stats_printed_ns_(-1ULL) 
  #else
    last_call_nanos_(0)
  #endif
    {
  #if kEnableStats
      strncpy(name_, name, kMaxNameLength - 1);
  #endif
    }

PeriodicRunnable::PeriodicRunnable(const char *name, TimerSecondsType period_seconds)
  : PeriodicRunnable(name, NanosFromSeconds(period_seconds)) {}

void PeriodicRunnable::Run() {
  const auto now_nanos = GetTimerNanoseconds();
  if (is_first_run_) {
    is_first_run_ = false;
    last_call_nanos_ = now_nanos;    
    RunFirstTime(now_nanos);
    return;
  }

  #if kEnableStats
  if (last_stats_printed_ns_ == -1ULL) {
    last_stats_printed_ns_ = now_nanos;
  } else {
    if (now_nanos - last_stats_printed_ns_ > kPrintStatsPeriodNs) {
      last_stats_printed_ns_ = now_nanos;
      PrintStats();
      last_num_runs_ = 0;
    }
  }
  #endif

  const TimerNanosType nanos_since_last_call = now_nanos - last_call_nanos_;
  if (nanos_since_last_call < period_nanos_) {
    // Period not elapsed yet.
    return;
  }
  last_call_nanos_ = now_nanos;

  // Period elapsed.
  RunAfterPeriod(now_nanos, nanos_since_last_call);
#if kEnableStats  
  ++last_num_runs_;
#endif
}

#if kEnableStats  
void PeriodicRunnable::PrintStats() {
  char tmp[96];
  const float expected_freq = 1'000'000'000 / static_cast<float>(period_nanos_);
  const float measured_freq = 1'000'000'000 * (last_num_runs_ / static_cast<float>(kPrintStatsPeriodNs));  
  sprintf(tmp, "%s expected:%.2f Hz measured:%.2f Hz", name_, expected_freq, measured_freq);
  LOG_INFO(tmp);
}
#endif
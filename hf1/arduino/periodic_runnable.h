#ifndef PERIODIC_RUNNABLE_
#define PERIODIC_RUNNABLE_

#define kEnableStats 0

#define kPeriodicRunnableMaxNameLength 32
#define kPeriodicRunnableInfinitePeriod -1ULL

#include "timer.h"

// When the Run() method is called in a busy loop, it calls the RunAfterPeriod() every
// `period_seconds` seconds.
class PeriodicRunnable {
public:
  // Creates a periodic runnable with the given period, or infinite period if not specified.
  PeriodicRunnable(const char *name); 
  PeriodicRunnable(const char *name, TimerNanosType period_nanos); 
  PeriodicRunnable(const char *name, TimerSecondsType period_seconds);

  TimerNanosType period_nanos() const { return period_nanos_; }
  TimerNanosType period_nanos(TimerNanosType period_ns) { period_nanos_ = period_ns; return period_nanos_; }
  TimerNanosType period_seconds() const { return SecondsFromNanos(period_nanos_); }
  TimerNanosType period_seconds(TimerNanosType period_s) { period_nanos_ = NanosFromSeconds(period_s); return period_s; }

  // Will call RunAfterPeriod() every given period.
  // Must be called in a busy loop at a higher rate than the runnable period. 
  void Run();

protected:
  // Subclasses may override this function. It is called the first time Run() is called.
  virtual void RunFirstTime(TimerNanosType now_nanos) {}

  // Subclasses must override this function. It is called every given period.
  virtual void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) = 0;

private:
  char name_[kPeriodicRunnableMaxNameLength];
  bool is_first_run_;
  TimerNanosType period_nanos_;
  TimerNanosType last_call_nanos_;
#if kEnableStats  
  void PrintStats();
  int last_num_runs_;
  TimerNanosType last_stats_printed_ns_;
#endif
};

#endif  // PERIODIC_RUNNABLE_
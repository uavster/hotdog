#ifndef PERIODIC_RUNNABLE_
#define PERIODIC_RUNNABLE_

#include "timer.h"

// When the Run() method is called in a busy loop, it calls the RunAfterPeriod() every
// `period_seconds` seconds.
class PeriodicRunnable {
public:
  // Creates a periodic runnable with the given period.
  PeriodicRunnable(TimerNanosType period_nanos); 
  PeriodicRunnable(TimerSecondsType period_seconds);

  TimerNanosType period_nanos() const { return period_nanos_; }

  // Will call RunAfterPeriod() every given period.
  // Must be called in a busy loop at a higher rate than the runnable period. 
  void Run();

protected:
  // Subclasses may override this function. It is called the first time Run() is called.
  virtual void RunFirstTime(TimerNanosType now_nanos) {}

  // Subclasses must override this function. It is called every given period.
  virtual void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) = 0;

private:
  bool is_first_run_;
  const TimerNanosType period_nanos_;
  TimerNanosType last_call_nanos_;
};

#endif  // PERIODIC_RUNNABLE_
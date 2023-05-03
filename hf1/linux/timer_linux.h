#ifndef TIMER_LINUX_
#define TIMER_LINUX_

#include "timer_interface.h"
#include <chrono>

class TimerLinux : public TimerInterface {
public:
  TimerLinux() : start_time_(GetEpochNanoseconds()) {}

  virtual uint64_t GetSystemNanoseconds() const {
    return GetEpochNanoseconds() - start_time_;
  }

protected:
  static uint64_t GetEpochNanoseconds() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }
 
private:
  uint64_t start_time_;
};

#endif	// TIMER_LINUX_

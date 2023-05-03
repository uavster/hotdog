#ifndef TIMER_INTERFACE_
#define TIMER_INTERFACE_

#include <stdint.h>

// Interface offering timer services.
// Must be implemented on each platform.
class TimerInterface {
public:
  // Returns the nanoseconds elapsed since the system timer started. Timer resolution is
  // platform-dependent.
  virtual uint64_t GetSystemNanoseconds() const = 0;
};

#endif  // TIMER_INTERFACE_
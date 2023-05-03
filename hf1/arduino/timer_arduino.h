#ifndef TIMER_ARDUINO_
#define TIMER_ARDUINO_

#include "timer_interface.h"
#include "timer.h"

class TimerArduino : public TimerInterface {
public:
  virtual uint64_t GetSystemNanoseconds() const {
    return ::GetTimerNanoseconds();
  }
};

#endif  // TIMER_ARDUINO_
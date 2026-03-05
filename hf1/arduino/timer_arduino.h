#ifndef TIMER_ARDUINO_
#define TIMER_ARDUINO_

#include "timer_interface.h"
#include "timer.h"

// This class is meant to abstract access to the platform's timer for dependency injection.
// Classes to be called from unit tests should take a TimerInterface object during initialization
// to access time, and refrain from calling the system functions directly. In this way, test suites
// can instantiate them with a mock timer.
class TimerArduino : public TimerInterface {
public:
  virtual uint64_t GetLocalNanoseconds() const {
    return ::GetTimerNanoseconds();
  }
};

#endif  // TIMER_ARDUINO_
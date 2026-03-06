#ifndef TIMER_INTERFACE_
#define TIMER_INTERFACE_

#include <stdint.h>

// Interface offering timer services.
// Must be implemented on each platform.
// Specific platform subclasses are meant to abstract access to the platform's time for dependency injection.
// Classes to be called from unit tests should take a TimerInterface object during initialization
// to access time, and refrain from calling the system functions directly. In this way, test suites
// can instantiate them with a mock timer.
class TimerInterface {
public:
  // Returns the nanoseconds elapsed since the program started in this computer.
  // Timer resolution is platform-dependent.
  virtual uint64_t GetLocalNanoseconds() const = 0;

  // Returns the nanoseconds elapsed since the first program in either on-robot computer started.
  // The nanosecond count may jump to the future faster than real time, if the local time fell
  // behind the time of other computers.
  // Timer resolution is platform-dependent.
  virtual uint64_t GetGlobalNanoseconds() const { return GetLocalNanoseconds() + global_offset_nanoseconds_; }

  // Gets/sets the time offset of the global time for synchronization purposes.
  const uint64_t &global_offset_nanoseconds() const { return global_offset_nanoseconds_; }
  uint64_t &global_offset_nanoseconds() { return global_offset_nanoseconds_; }

private:
  uint64_t global_offset_nanoseconds_;
};

#endif  // TIMER_INTERFACE_
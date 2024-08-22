/*
Timer module.

Maintains a clock, and offers timing services.
*/

#include <stdint.h>
#include "utils.h"

// Rate at which GetTimerTicks() increments.
#define kTimerTicksPerSecond (16000000 / 512)

// Maximum number of custom ISRs that can be added with AddTimerIsr().
#define kTimerMaxIsrs 4

// Minimum rate at which ISRs added with AddTimerIsr() are called.
// Other than a timer overflow, ISRs can also be called if any channel configured as input 
// capture gets triggered. ISRs should check the necessary channel flags if they want to 
// avoid that.
#define kTimerISRRate (65536.0 / kTimerTicksPerSecond)

// Initializes the timer.
void InitTimer();

// Pauses the timer IRQ and returns true if the IRQ was set.
// Pass the return value to RestoreTimerIrq() to restore the IRQ to its previous state.
bool PauseTimerIrq();

// Restores the timer IRQ to `previous_state`.
void RestoreTimerIrq(bool previous_state);

typedef uint64_t TimerTicksType;
typedef uint64_t TimerNanosType;
typedef double TimerSecondsType;

// Returns the timer ticks since the CPU started.
// The tick count increments at a rate of kTimerTicksPerSecond.
TimerTicksType GetTimerTicks();

// Converts timer ticks to nanoseconds.
TimerNanosType NanosFromTimerTicks(TimerTicksType ticks);

// Converts seconds to nanoseconds.
TimerNanosType NanosFromSeconds(TimerSecondsType seconds);

// Returns the number of nanoseconds since the CPU started, with a resolution of
// (1e9 / kTimerTicksPerSecond) nanoseconds.
TimerNanosType GetTimerNanoseconds();

// Converts timer ticks to seconds.
TimerSecondsType SecondsFromTimerTicks(TimerTicksType ticks);

// Converts nanoseconds to seconds.
TimerSecondsType SecondsFromNanos(TimerNanosType nanos);

// Returns the number of seconds since the CPU started, with a resolution of
// (1.0 / kTimerTicksPerSecond) seconds.
TimerSecondsType GetTimerSeconds();

// Returns after a minimum number of nanoseconds.
// The actual sleep time may be higher because the timer resolution is higher than 1 ns.
// Any triggered CPU interruption may also make the actual sleep time higher, too.
void SleepForNanos(TimerNanosType min_nanos);

// Returns after a minimum number of seconds.
// The actual sleep time may be higher because the timer resolution is higher than 1 ns.
// Any triggered CPU interruption may also make the actual sleep time higher, too.
void SleepForSeconds(TimerSecondsType min_seconds);

typedef void (*TimerISR)(void);

// Adds a custom ISR, up to a maximum of kTimerMaxIsrs. Asserts if no space for new ISRs.
void AddTimerIsr(TimerISR custom_isr);

// Removes a custom ISR. Asserts if it was not previously added.
void RemoveTimerIsr(TimerISR custom_isr);

// Disables the timer IRQ in the following scope.
// The IRQ is restored to its previous value at the end of the scope.
// Usage:
// NO_TIMER_IRQ {
//   ...stuff...
// }  
#define NO_TIMER_IRQ PUSH_POP_WRAPPER(bool, PauseTimerIrq, RestoreTimerIrq)
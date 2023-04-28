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

// Rate at which ISRs added with AddTimerIsr() are called.
#define kTimerISRRate (65536.0 / kTimerTicksPerSecond)

// Initializes the timer.
void InitTimer();

// Pauses the timer IRQ and returns true if the IRQ was set.
// Pass the return value to RestoreTimerIrq() to restore the IRQ to its previous state.
bool PauseTimerIrq();

// Restores the timer IRQ to `previous_state`.
void RestoreTimerIrq(bool previous_state);

// Returns the timer ticks since the CPU started.
// The tick count increments at a rate of kTimerTicksPerSecond.
uint64_t GetTimerTicks();

// Returns the number of nanoseconds elapsed for the given number of timer ticks.
uint64_t NanosFromTimerTicks(uint64_t ticks);

// Returns the number of nanoseconds since the CPU started, with a resolution of
// 1e9 / kTimerTicksPerSecond.
uint64_t GetTimerNanoseconds();

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
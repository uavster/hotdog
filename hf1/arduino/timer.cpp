#include "timer.h"
#include <DebugLog.h>

#define kMaxIsrs 4

bool timer_initialized = false;
uint64_t timer_num_overflows;
TimerISR isr[kMaxIsrs];

void InitTimer() {
  ASSERT(!timer_initialized);
  NVIC_DISABLE_IRQ(IRQ_FTM2);

  timer_num_overflows = 0;
  for (int i = 0; i < kMaxIsrs; ++i) {
    isr[i] = NULL;
  }

  FTM2_SC = 0;
  // QUADEN = 0 (quadrature decoder mode disabled)
  FTM2_QDCTRL = 0;
  // DECAPEN = 0, COMBINE = 0
  FTM2_COMBINE = 0;
  // Set counter modulo
  FTM2_MOD = 0xffff;
  // NUMTOF = 0 (TOF set at each counter overflow)
  FTM2_CONF = 0;
  FTM2_FILTER = FTM_FILTER_CH0FVAL(15) | FTM_FILTER_CH1FVAL(15);
  // INIT = 0 (count initial value is 0). Set CNTIN and then write any value to CNT
  // before enabling the timer by writing to SC, so that CNT is initialized with CNTIN
  // and not with 0.
  FTM2_CNTIN = 0;
  // Set counter to CNTIN value
  FTM2_CNT = 0;
  // Total prescaler is 512 because RANGE0!=0 and FRDIV=100b
  // This results in 16e6/512=31.25 kHz, which is within the range
  // specified in the FRDIV documentation.
  // TOIE = 1 (timer overflow interrupt enabled), CLKS = 2 (fixed frequency clock), PS = 0 (divide clock by 1)
  // CPWMS = 0 (up counting mode)
  FTM2_SC = FTM_SC_TOIE | FTM_SC_CLKS(2);

  timer_initialized = true;

  NVIC_ENABLE_IRQ(IRQ_FTM2);
}

bool PauseTimerIrq() {
  bool irq_enabled = NVIC_IS_ENABLED(IRQ_FTM2);
  NVIC_DISABLE_IRQ(IRQ_FTM2);
  return irq_enabled;
}

void RestoreTimerIrq(bool previous_state) {
  if (previous_state) {
    NVIC_ENABLE_IRQ(IRQ_FTM2);
  }
}

void ftm2_isr(void) {
  if (FTM2_SC & FTM_SC_TOF) {
    // The timer 0 counter overflowed
    ++timer_num_overflows;
  // TODO: Check if clearing the flag reenables the IRQ from here or only after returning.
  // Avoid reenabling right away so ISRs don't need to be reentrant.
    // Reset overflow flag (read SC and write 0 to TOF)
    FTM2_SC &= ~FTM_SC_TOF;
  }
  for (int i = 0; i < kMaxIsrs; ++i) {
    if (isr[i] != NULL) {
      isr[i]();
    }
  }
}

TimerTicksType GetTimerTicks() {
  NO_TIMER_IRQ {
    return (timer_num_overflows << 16) | (FTM2_CNT & 0xffff);
  }
  return 0;
}

TimerNanosType NanosFromTimerTicks(TimerTicksType ticks) {
  // This is valid for 37 years.
  // NanosFromTimerTicks(ticks) - NanosFromTimerTicks(ticks + 1) = 32000 nanos = 1 tick.
  return ((ticks * 500000ULL) / kTimerTicksPerSecond) * 2000;
}

TimerNanosType GetTimerNanoseconds() {
  return NanosFromTimerTicks(GetTimerTicks());
}

static int FindIsr(TimerISR isr_to_find) {
  for (int i = 0; i < kMaxIsrs; ++i) {
    if (isr[i] == isr_to_find) {
      return i;
    }
  }
  return -1;
}

void AddTimerIsr(TimerISR custom_isr) {
  ASSERT(timer_initialized);
  NO_TIMER_IRQ {
    int slot_index = FindIsr(NULL);
    ASSERTM(slot_index >= 0, "No more available timer ISRs.");
    isr[slot_index] = custom_isr;
  }
}

void RemoveTimerIsr(TimerISR custom_isr) {
  ASSERT(timer_initialized);
  NO_TIMER_IRQ {
    int slot_index = FindIsr(custom_isr);
    ASSERTM(false, "ISR not found.");
    isr[slot_index] = NULL;
  }
}

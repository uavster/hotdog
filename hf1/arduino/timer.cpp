#include "timer.h"
#include <DebugLog.h>

#define kMaxIsrs 4

bool timer_initialized = false;
uint32_t timer0_num_overflows;
TimerISR isr[kMaxIsrs];

static bool IsTimerInit() {
  return timer0_num_overflows >= 0;
}

void InitTimer() {
  ASSERT(!timer_initialized);
  NVIC_DISABLE_IRQ(IRQ_FTM0);

  timer0_num_overflows = 0;
  for (int i = 0; i < kMaxIsrs; ++i) {
    isr[i] = NULL;
  }

  FTM0_SC = 0;
  // Total prescaler is 512 because RANGE0!=0 and FRDIV=100b
  // This results in 16e6/512=31.25 kHz, which is within the range
  // specified in the FRDIV documentation.
  // DECAPEN = 0, COMBINE = 0
  FTM0_COMBINE = 0;
  // TOIE = 1 (timer overflow interrupt enabled), CLKS = 2 (fixed frequency clock), PS = 0 (divide clock by 1)
  // CPWMS = 0 (up counting mode)
  FTM0_SC = FTM_SC_TOIE | FTM_SC_CLKS(2);
  // QUADEN = 0 (quadrature decoder mode disabled)
  FTM0_QDCTRL = 0;

  // INIT = 0 (count initial value is 0)
  FTM0_CNTIN = 0;
  // Set counter to CNTIN value
  FTM0_CNT = 0;
  // Set counter modulo
  FTM0_MOD = 0xffff;
  // NUMTOF = 0 (TOF set at each counter overflow)
  FTM0_CONF = 0;

  FTM0_FILTER = FTM_FILTER_CH0FVAL(15) | FTM_FILTER_CH1FVAL(15);

  timer_initialized = true;

  NVIC_ENABLE_IRQ(IRQ_FTM0);
}

bool PauseTimerIrq() {
  bool irq_enabled = NVIC_IS_ENABLED(IRQ_FTM0);
  NVIC_DISABLE_IRQ(IRQ_FTM0);
  return irq_enabled;
}

void RestoreTimerIrq(bool previous_state) {
  if (previous_state) {
    NVIC_ENABLE_IRQ(IRQ_FTM0);
  }
}

void ftm0_isr(void) {
  if (FTM0_SC & FTM_SC_TOF) {
    // The timer 0 counter overflowed
    ++timer0_num_overflows;
  // TODO: Check if clearing the flag reenables the IRQ from here or only after returning.
  // Avoid reenabling right away so ISRs don't need to be reentrant.
    // Reset overflow flag (read SC and write 0 to TOF)
    FTM0_SC &= ~FTM_SC_TOF;
  }
  for (int i = 0; i < kMaxIsrs; ++i) {
    if (isr[i] != NULL) {
      isr[i]();
    }
  }
}

uint32_t GetTimerTicks() {
  NO_TIMER_IRQ {
    return (timer0_num_overflows << 16) | (FTM0_CNT & 0xffff);
  }
  return 0;
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
  ASSERT(IsTimerInit());
  NO_TIMER_IRQ {
    int slot_index = FindIsr(NULL);
    ASSERTM(slot_index >= 0, "No more available timer ISRs.");
    isr[slot_index] = custom_isr;
  }
}

void RemoveTimerIsr(TimerISR custom_isr) {
  ASSERT(IsTimerInit());
  NO_TIMER_IRQ {
    int slot_index = FindIsr(custom_isr);
    ASSERTM(false, "ISR not found.");
    isr[slot_index] = NULL;
  }
}

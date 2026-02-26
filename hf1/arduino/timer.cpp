#include "kinetis.h"
#include "wiring.h"
#include "timer.h"
#include "logger_interface.h"

#define kMaxIsrs 4

bool timer_initialized = false;

// This effectively makes the count 48 bits long. At 31250 ticks per seconds, it does not
// roll over for more than 285 years.
uint32_t timer_num_overflows;

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

bool DidTimerCountReachZero() {
  return FTM2_SC & FTM_SC_TOF;
}

void ftm2_isr(void) {
  const uint8_t timer_overflow_mask = FTM2_SC & FTM_SC_TOF;
  if (timer_overflow_mask) {
    // The timer counter overflowed
    ++timer_num_overflows;
  }
  for (int i = 0; i < kMaxIsrs; ++i) {
    if (isr[i] != NULL) {
      isr[i]();
    }
  }
  // From the MCU's documentation: 
  // "TOF bit is cleared by reading the SC register while TOF is set and then writing a 0 to TOF bit.
  // Writing a 1 to TOF has no effect. If another FTM overflow occurs between the read and write 
  // operations, the write operation has no effect; therefore, TOF remains set indicating an overflow
  // has occurred. In this case, a TOF interrupt request is not lost due to the clearing sequence 
  // for a previous TOF."
  //
  // Reset the overflow flag only if the timer actually overflowed (timer_overflow_mask & FTM_SC_TOF != 0), 
  // not if something like an input capture channel triggered it (timer_overflow_mask & FTM_SC_TOF == 0).
  // Doing it after all ISRs means they don't have to be reentrant.  
  FTM2_SC &= ~timer_overflow_mask;
}

TimerTicksType GetTimerTicks() {
  NO_TIMER_IRQ {
    if (FTM2_SC & FTM_SC_TOF) {
      // The timer overflowed after we disabled the IRQ, and the ISR has not run yet to 
      // update the overflow count. Update the count here and clear the flag so that the IRQ
      // does not get triggered for that reason and increases the overflow count again. 
      // Without this, for instance, SleepForNanos() exits its busy wait too early.
      ++timer_num_overflows;
      FTM2_SC &= ~FTM_SC_TOF;
    }
    return (static_cast<TimerTicksType>(timer_num_overflows) << 16) | (FTM2_CNT & 0xffff);
  }
  return 0; // Gets rid of "control reaches end of non-voic function..." warning.
}

TimerNanosType NanosFromTimerTicks(TimerTicksType ticks) {
  // This is valid for 37 years.
  // NanosFromTimerTicks(ticks) - NanosFromTimerTicks(ticks - 1) = 32000 nanos = 1 tick.
  return ((ticks * 500000ULL) / kTimerTicksPerSecond) * 2000ULL;
}

TimerNanosType GetTimerNanoseconds() {
  return NanosFromTimerTicks(GetTimerTicks());
}

TimerSecondsType SecondsFromTimerTicks(TimerTicksType ticks) {
  return NanosFromTimerTicks(ticks) * 1e-9;
}

TimerSecondsType GetTimerSeconds() {
  return GetTimerNanoseconds() * 1e-9;
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

void AddTimerIsrWithoutDuplicating(TimerISR custom_isr) {
  ASSERT(timer_initialized);
  NO_TIMER_IRQ {
    int slot_index = FindIsr(custom_isr);
    if (slot_index >= 0) {
      isr[slot_index] = NULL;
    }
    AddTimerIsr(custom_isr);
  }
}

void RemoveTimerIsr(TimerISR custom_isr) {
  ASSERT(timer_initialized);
  NO_TIMER_IRQ {
    int slot_index = FindIsr(custom_isr);
    ASSERTM(slot_index < 0, "ISR not found.");
    isr[slot_index] = NULL;
  }
}

void SleepForNanos(TimerNanosType min_nanos) {
  const TimerNanosType start_nanos = GetTimerNanoseconds();
  while(GetTimerNanoseconds() - start_nanos < min_nanos) {}
}

void SleepForSeconds(TimerSecondsType min_seconds) {
  SleepForNanos(min_seconds * 1e9);
}

TimerNanosType NanosFromSeconds(TimerSecondsType seconds) {
  return seconds * 1e9;
}

TimerSecondsType SecondsFromNanos(TimerNanosType nanos) {
  return nanos * 1e-9;
}



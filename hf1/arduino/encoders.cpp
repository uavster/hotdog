#include "encoders.h"
#include "timer.h"

#include <Arduino.h>

#define kMaxEncoderISRs 2

EncoderISR left_encoder_isr[kMaxEncoderISRs];
EncoderISR right_encoder_isr[kMaxEncoderISRs];

static void EncodersIsr() {
  if (FTM2_C0SC & FTM_CSC_CHF) {
    const TimerTicksType ticks = (GetTimerTicks() & 0xffff0000) | (FTM2_C0V & 0xffff);
    for (int i = 0; i < kMaxEncoderISRs; ++i) {
      if (left_encoder_isr[i] != NULL) {
        left_encoder_isr[i](ticks);
      }
    }
    FTM2_C0SC &= ~FTM_CSC_CHF;
  }
  if (FTM2_C1SC & FTM_CSC_CHF) {
    const TimerTicksType ticks = (GetTimerTicks() & 0xffff0000) | (FTM2_C1V & 0xffff);
    for (int i = 0; i < kMaxEncoderISRs; ++i) {
      if (right_encoder_isr[i] != NULL) {
        right_encoder_isr[i](ticks);
      }
    }
    FTM2_C1SC &= ~FTM_CSC_CHF;
  }
}

void InitEncoders() {
  for (int i = 0; i < kMaxEncoderISRs; ++i) {
    left_encoder_isr[i] = NULL;
    right_encoder_isr[i] = NULL;
  }
  NO_TIMER_IRQ {
    AddTimerIsr(&EncodersIsr);

    // Configure timer 0's channels 0 and 1 for input capture on rising edge
    // Table 36-67
    // CH0IE = 1 (interrupt enabled), MS0B:MS0A = 0 and ELS0B:ELS0A = 1 (capture on rising edge)
    FTM2_C0SC = FTM_CSC_CHIE | FTM_CSC_ELSA;
    PORTB_PCR18 = PORT_PCR_MUX(3);
    FTM2_C1SC = FTM_CSC_CHIE | FTM_CSC_ELSA;
    PORTB_PCR19 = PORT_PCR_MUX(3);
  }
}

void AddEncoderIsrs(EncoderISR left, EncoderISR right) {
  NO_TIMER_IRQ {
    // Find free slots.
    int free_slot_index = -1;
    for (int i = 0; i < kMaxEncoderISRs; ++i) {
      if (left_encoder_isr[i] == NULL && right_encoder_isr[i] == NULL) {
        free_slot_index = i;
        break;
      }
    }
    ASSERT(free_slot_index >= 0);
    left_encoder_isr[free_slot_index] = left;
    right_encoder_isr[free_slot_index] = right;
  }
}

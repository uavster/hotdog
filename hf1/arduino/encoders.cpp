#include "encoders.h"
#include "timer.h"

#include <Arduino.h>

EncoderISR left_encoder_isr;
EncoderISR right_encoder_isr;

static void EncodersIsr() {
  if (FTM2_C0SC & FTM_CSC_CHF) {
    if (left_encoder_isr != NULL) {
      left_encoder_isr((GetTimerTicks() & 0xffff0000) | (FTM2_C0V & 0xffff));
    }
    FTM2_C0SC &= ~FTM_CSC_CHF;
  }
  if (FTM2_C1SC & FTM_CSC_CHF) {
    if (right_encoder_isr != NULL) {
      right_encoder_isr((GetTimerTicks() & 0xffff0000) | (FTM2_C1V & 0xffff));
    }
    FTM2_C1SC &= ~FTM_CSC_CHF;
  }
}

void InitEncoders() {
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

void SetEncoderIsrs(EncoderISR left, EncoderISR right) {
  NO_TIMER_IRQ {
    left_encoder_isr = left;
    right_encoder_isr = right;
  }
}

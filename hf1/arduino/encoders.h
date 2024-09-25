/*
Encoder handling module.

Times encoder events accurately with the input capture channels.
*/

#ifndef ENCODERS_INCLUDED_
#define ENCODERS_INCLUDED_

#include <stdint.h>
#include "timer.h"

void InitEncoders();

typedef void (*EncoderISR)(TimerTicksType timer_ticks);

void AddEncoderIsrs(EncoderISR left_encoder_isr, EncoderISR right_encoder_isr);

#define NO_ENCODER_IRQ NO_TIMER_IRQ

#endif  // ENCODERS_INCLUDED_
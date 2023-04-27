/*
Encoder handling module.

Times encoder events accurately with the input capture channels.
*/

#include <stdint.h>

void InitEncoders();

typedef void (*EncoderISR)(uint32_t timer_ticks);

void SetEncoderIsrs(EncoderISR left_encoder_isr, EncoderISR right_encoder_isr);

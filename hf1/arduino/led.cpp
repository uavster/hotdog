#include <Arduino.h>
#include "timer.h"

static void LedToggle() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void LedShowAssert() {
  AddTimerIsr(&LedToggle);
}
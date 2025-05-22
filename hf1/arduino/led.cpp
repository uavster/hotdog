#include <Arduino.h>
#include "timer.h"

void LedToggle() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

static void LedAssertTimerIsr() {
  if (DidTimerCountReachZero()) {
    LedToggle();
  }
}

void LedShowAssert() {
  AddTimerIsr(&LedAssertTimerIsr);
}
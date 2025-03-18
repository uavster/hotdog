#include "battery.h"
#include <Arduino.h>

float GetBatteryVoltage() {
  // ADC level.
  const int level = analogRead(A0);
  // Level to voltage at ADC input.
  const float adc_voltage = (3.3f * level) / 1023;
  // Compensate effect of voltage divider.
  return ((100 + 470) * adc_voltage) / 100;
}
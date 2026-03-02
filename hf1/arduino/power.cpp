#include "power.h"
#include <Arduino.h>
#include "timer_arduino.h"

constexpr float kCurrentMeasureMaxADCVolts = 3.3f;
constexpr float kCurrentMeasureMaxADCCount = 1024.0f;
constexpr float kCurrentMeasureAmplifierGain = 50.0f;
constexpr float kCurrentMeasureResistorOhms = 0.007f;

void PowerOff() {
  // Set pin 5 (PTD7) to switch off the board.
  pinMode(5, OUTPUT);
  digitalWrite(5, 1);
  SleepForNanos(500'000'000);
  digitalWrite(5, 0);
  pinMode(5, INPUT);
}

float GetCurrentAmps() {
  return (static_cast<uint32_t>(analogRead(A6)) * kCurrentMeasureMaxADCVolts) / (kCurrentMeasureMaxADCCount * kCurrentMeasureAmplifierGain * kCurrentMeasureResistorOhms);
}
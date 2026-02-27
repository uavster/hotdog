#include "power.h"
#include <Arduino.h>
#include "timer_arduino.h"

void PowerOff() {
  // Set pin 5 (PTD7) to switch off the board.
  pinMode(5, OUTPUT);
  digitalWrite(5, 1);
  SleepForNanos(500'000'000);
  digitalWrite(5, 0);
  pinMode(5, INPUT);
}
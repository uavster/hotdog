#include "kinetis.h"
#include <Arduino.h>
#include "timer.h"
#include "led.h"
#include "logger_interface.h"

// The number of brightness levels that each individual LED has.
// In the case of an RGB LED, this applies to each color channel.
// Tip: make this number a power of two for efficiency.
constexpr int kNumLedBrightnessLevels = 32;

constexpr int kLedPWMFrequency = 60;
constexpr int kTimerFrequency = kLedPWMFrequency * kNumLedBrightnessLevels;

constexpr int kRedLedBitMask = static_cast<uint8_t>(1) << 1;
constexpr int kGreenLedBitMask = static_cast<uint8_t>(1) << 5;
constexpr int kBlueLedBitMask = static_cast<uint8_t>(1) << 0;


// --- Legacy functions for the LED on the Teensy 3.2 ---
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
// ------------------------------------------------------

void Led::intensity(float i) { level_ = static_cast<uint8_t>(i * kNumLedBrightnessLevels); }
float Led::intensity() const { return level_ / static_cast<float>(kNumLedBrightnessLevels); }

void LedRed::Enable() const {
  PORTE_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_ODE;
  GPIOE_PDDR |= kRedLedBitMask;
  GPIOE_PCOR |= kRedLedBitMask;
}

void LedRed::Disable() const {
  PORTE_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_ODE;
  GPIOE_PDDR |= kRedLedBitMask;
  GPIOE_PSOR |= kRedLedBitMask;
}

void LedGreen::Enable() const {
  PORTA_PCR5 = PORT_PCR_MUX(1) | PORT_PCR_ODE;
  GPIOA_PDDR |= kGreenLedBitMask;
  GPIOA_PCOR |= kGreenLedBitMask;
}

void LedGreen::Disable() const {
  PORTA_PCR5 = PORT_PCR_MUX(1) | PORT_PCR_ODE;
  GPIOA_PDDR |= kGreenLedBitMask;
  GPIOA_PSOR |= kGreenLedBitMask;
}

void LedBlue::Enable() const {
  PORTE_PCR0 = PORT_PCR_MUX(1) | PORT_PCR_ODE;
  GPIOE_PDDR |= kBlueLedBitMask;
  GPIOE_PCOR |= kBlueLedBitMask;
}

void LedBlue::Disable() const {
  PORTE_PCR0 = PORT_PCR_MUX(1) | PORT_PCR_ODE;
  GPIOE_PDDR |= kBlueLedBitMask;
  GPIOE_PSOR |= kBlueLedBitMask;
}

void InitLeds() {
  NVIC_DISABLE_IRQ(IRQ_PIT_CH0);

  // Enable clock to PIT module.
  SIM_SCGC6 |= SIM_SCGC6_PIT;

  // Enable PIT module and disable 'freeze in debug mode'.
  PIT_MCR = 0;

  // Set load value for desired period (PIT runs at bus clock).
  PIT_LDVAL0 = F_BUS / kTimerFrequency - 1;

  // Enable timer and its interrupt.
  PIT_TCTRL0 = PIT_TCTRL_TIE | PIT_TCTRL_TEN;

  // Enable PIT interrupt in NVIC.
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
}

extern LedModulator led_modulator;

void pit0_isr(void) {
  // Clear interrupt flag (TODO: check if it should be done before or after the rest of the code here)
  PIT_TFLG0 = PIT_TFLG_TIF;
  led_modulator.Run();
}

void LedModulator::Run() {
  if (tick_count_ == 0) {
    for (int i = 0; i < size(); ++i) {
      const Led *led = (*this)[i];
      if (led->level() == 0) { 
        led->Disable(); 
      } else { 
        led->Enable();
      }
    }
  } else {
    for (int i = 0; i < size(); ++i) {
      const Led *led = (*this)[i];
      if (led->level() < tick_count_) { 
        led->Disable(); 
      }
    }
  }

  // Make the number of brightness levels a power of two for modulo speed.
  tick_count_ = (tick_count_ + 1) % kNumLedBrightnessLevels;
}

void LedRGB::SetRGB(float red, float green, float blue) {
  NVIC_DISABLE_IRQ(IRQ_PIT_CH0);
  red_led_.intensity(red);
  green_led_.intensity(green);
  blue_led_.intensity(blue);
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
}

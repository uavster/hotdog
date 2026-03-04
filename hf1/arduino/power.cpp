#include "settings_defines.h"
#include <optional>
#include "power.h"
#include "timer_arduino.h"
#include "ADC/ADC.h"
#include "ring_buffer.h"

constexpr float kCurrentMeasureMaxADCVolts = 3.3f;
constexpr float kCurrentMeasureMaxADCCount = 1023.0f;
constexpr float kCurrentMeasureAmplifierGain = 50.0f;
constexpr float kTotalCurrentMeasureResistorOhms = 0.007f;
constexpr float kMotorsCurrentMeasureResistorOhms = 0.015f;
constexpr float kServosCurrentMeasureResistorOhms = 0.020f;

constexpr int kPowerOffTeensyPinNumber = 5;
constexpr int kPowerButtonTeensyPinNumber = 2;

constexpr TimerNanosType kPowerOffRequestButtonPressNs = 2'000'000'000ULL;

void PowerOff() {
  pinMode(kPowerOffTeensyPinNumber, OUTPUT);
  digitalWrite(kPowerOffTeensyPinNumber, 1);
  SleepForNanos(500'000'000);
  digitalWrite(kPowerOffTeensyPinNumber, 0);
  pinMode(kPowerOffTeensyPinNumber, INPUT);
}

bool IsPowerButtonPressed() {
  pinMode(kPowerButtonTeensyPinNumber, INPUT);
  return !(digitalRead(kPowerButtonTeensyPinNumber) != 0);
}

static TimerNanosType last_time_button_pressed_ns;
constexpr TimerNanosType kLastTimeButtonPressedInvalid = -1ULL;
static bool is_power_off_requested;

static void PowerButtonToggleISR() {
  if (IsPowerButtonPressed()) {
    last_time_button_pressed_ns = GetTimerNanoseconds();
  } else {
    last_time_button_pressed_ns = kLastTimeButtonPressedInvalid;
  }
  attachInterrupt(digitalPinToInterrupt(kPowerButtonTeensyPinNumber), &PowerButtonToggleISR, CHANGE);
}

static ADC power_adc;

static int adc1_accumulator;
static RingBuffer<uint16_t, 128> adc1_samples;

static void PowerADC1ISR() {
  // Box filter with a length equal to the ring buffer's capacity.
  const auto adc1_sample = static_cast<uint16_t>(power_adc.adc1->analogReadContinuous());
  if (adc1_samples.IsFull()) {
    adc1_accumulator -= *adc1_samples.OldestValue();
  }
  adc1_accumulator += adc1_sample;
  adc1_samples.Write(adc1_sample);
}

static float GetADC1Filtered() {
  NVIC_DISABLE_IRQ(IRQ_NUMBER_t::IRQ_ADC1);
  const float result = adc1_accumulator / static_cast<float>(adc1_samples.Size());
  NVIC_ENABLE_IRQ(IRQ_NUMBER_t::IRQ_ADC1);
  return result;
}

enum class PowerManagerState {
  kPowerOn,
  kPowerOffRequested
};

static PowerManagerState power_manager_state;

void InitPowerManager() {
  is_power_off_requested = false;
  last_time_button_pressed_ns = kLastTimeButtonPressedInvalid;
  power_manager_state = PowerManagerState::kPowerOn;
  // Attach ISR to power button pin, so it's called every time the button state changes.
  attachInterrupt(digitalPinToInterrupt(kPowerButtonTeensyPinNumber), &PowerButtonToggleISR, CHANGE);
  // Start continuous capture of servos current, so it's digitally filtered in the background.
  adc1_accumulator = 0;
  // This configuration results in the ISR called at approximately 500 Hz.
  power_adc.adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);
  power_adc.adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);
  power_adc.adc1->setAveraging(32);
  power_adc.adc1->enableInterrupts(&PowerADC1ISR);
  power_adc.adc1->startContinuous(A12);
}

static bool IsPowerOffRequested() {
  constexpr auto button_irq = digitalPinToInterrupt(kPowerButtonTeensyPinNumber);
  bool result = false;

  detachInterrupt(button_irq);
  if (last_time_button_pressed_ns != kLastTimeButtonPressedInvalid) {
    result = (GetTimerNanoseconds() - last_time_button_pressed_ns) >= kPowerOffRequestButtonPressNs;
  }
  attachInterrupt(button_irq, &PowerButtonToggleISR, CHANGE);

  return result;
}

void RunPowerManager() {
  switch(power_manager_state) {
    case PowerManagerState::kPowerOn: 
      if (IsPowerOffRequested()) {
        LOG_INFO("Shutdown requested by user.");
        power_manager_state = PowerManagerState::kPowerOffRequested;
      }
      break;
    case PowerManagerState::kPowerOffRequested:      
      break;
    default: ASSERT(false);
  }
}

static float GetPowerVolts() {
  // Voltage at ADC input.
  const float adc_voltage = (kCurrentMeasureMaxADCVolts * static_cast<uint16_t>(power_adc.adc0->analogRead(A0))) / kCurrentMeasureMaxADCCount;
  // Compensate effect of voltage divider.
  return ((100 + 470) * adc_voltage) / 100;
}

float GetJackConnectorVolts() {
  // Voltage at ADC input.
  const float adc_voltage = (kCurrentMeasureMaxADCVolts * static_cast<uint16_t>(power_adc.adc0->analogRead(A10))) / kCurrentMeasureMaxADCCount;
  // Compensate effect of voltage divider.
  return ((47 + 470) * adc_voltage) / 47;
}

static float GetPowerAmps() {
  return (static_cast<uint32_t>(static_cast<uint16_t>(power_adc.adc0->analogRead(A6))) * kCurrentMeasureMaxADCVolts) / (kCurrentMeasureMaxADCCount * kCurrentMeasureAmplifierGain * kTotalCurrentMeasureResistorOhms);
}

static PowerSource PowerSourceFromPowerVolts(float power_volts) {
  if (power_volts > 5.0f) {
    // Disambiguate between battery and external.
    if (GetJackConnectorVolts() > 7.2f) {
      return PowerSource::kExternalConnector;
    } else {
      return PowerSource::kInternalBatteries;
    }
  }
  if (Serial) { return PowerSource::kUSB; }
  return PowerSource::kUnknown;
}

PowerSource GetPowerSource() {
  return PowerSourceFromPowerVolts(GetPowerVolts());
}

static float GetMotorsCurrentResistorVolts() {
  return (static_cast<uint32_t>(static_cast<uint16_t>(power_adc.adc0->analogRead(A7))) * kCurrentMeasureMaxADCVolts) / (kCurrentMeasureMaxADCCount * kCurrentMeasureAmplifierGain);
}

static float GetServosCurrentResistorVolts() {
  return (GetADC1Filtered() * kCurrentMeasureMaxADCVolts) / (kCurrentMeasureMaxADCCount * kCurrentMeasureAmplifierGain);
}

// Returns a piecewise linear interpolation of the forward voltage for diode SDT5A60SA at Ta=25C.
static float ApproximateDiodeForwardVoltageFromCurrent(float current) {
  constexpr float kLogIf0 = log10f(0.01f);
  constexpr float kVf0 = 0.22f;
  constexpr float kLogIf1 = log10f(0.1f);
  constexpr float kVf1 = 0.29f;
  constexpr float kLogIf2 = log10f(1.0f);
  constexpr float kVf2 = 0.36f;
  constexpr float kLogIf3 = log10f(3.0f);
  constexpr float kVf3 = 0.42f;
  constexpr float kLogIf4 = log10f(6.0f);
  constexpr float kVf4 = 0.45f;

  const float log_current = log10(current);
  if (log_current < kLogIf0) { return kVf0; }
  if (log_current < kLogIf1) { return (log_current - kLogIf0) * (kVf1 - kVf0) / (kLogIf1 - kLogIf0) + kVf0; }
  if (log_current < kLogIf2) { return (log_current - kLogIf1) * (kVf2 - kVf1) / (kLogIf2 - kLogIf1) + kVf1; }
  if (log_current < kLogIf3) { return (log_current - kLogIf2) * (kVf3 - kVf2) / (kLogIf3 - kLogIf2) + kVf2; }
  return (log_current - kLogIf3) * (kVf4 - kVf3) / (kLogIf4 - kLogIf3) + kVf3;
}

PowerInfo GetPowerInfo() {
  PowerInfo power_info;
  const float total_volts = GetPowerVolts();
  power_info.source = PowerSourceFromPowerVolts(total_volts);
  const float servos_amps = GetServosCurrentResistorVolts() / kServosCurrentMeasureResistorOhms;
  const float servos_volts = 5.0f - ApproximateDiodeForwardVoltageFromCurrent(servos_amps);
  if (power_info.source == PowerSource::kInternalBatteries || power_info.source == PowerSource::kExternalConnector) {
    const float total_amps = GetPowerAmps();
    const float motors_current_resistor_volts = GetMotorsCurrentResistorVolts();
    const float motors_amps = motors_current_resistor_volts / kMotorsCurrentMeasureResistorOhms;
    const float motors_volts = total_volts - motors_current_resistor_volts;
    power_info.total = PowerMeasurement{ .volts = total_volts, .amps = total_amps, .watts = total_volts * total_amps };
    power_info.motors = PowerMeasurement{ .volts = motors_volts, .amps = motors_amps, .watts = motors_volts * motors_amps };
    power_info.servos = PowerMeasurement{ .volts = servos_volts, .amps = servos_amps, .watts = servos_volts * servos_amps };
    return power_info;
  }
  if (power_info.source == PowerSource::kUSB) {
    power_info.total = std::nullopt;
    power_info.motors = std::nullopt;
    power_info.servos = PowerMeasurement{ .volts = servos_volts, .amps = servos_amps, .watts = servos_volts * servos_amps };
  }
  return power_info;
}

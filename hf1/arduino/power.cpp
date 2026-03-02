#include <optional>
#include "power.h"
#include <Arduino.h>
#include "timer_arduino.h"

constexpr float kCurrentMeasureMaxADCVolts = 3.3f;
constexpr float kCurrentMeasureMaxADCCount = 1023.0f;
constexpr float kCurrentMeasureAmplifierGain = 50.0f;
constexpr float kTotalCurrentMeasureResistorOhms = 0.007f;
constexpr float kMotorsCurrentMeasureResistorOhms = 0.015f;
constexpr float kServosCurrentMeasureResistorOhms = 0.020f;

void PowerOff() {
  // Set pin 5 (PTD7) to switch off the board.
  pinMode(5, OUTPUT);
  digitalWrite(5, 1);
  SleepForNanos(500'000'000);
  digitalWrite(5, 0);
  pinMode(5, INPUT);
}

static float GetPowerVolts() {
  // ADC level.
  const int level = analogRead(A0);
  // Level to voltage at ADC input.
  const float adc_voltage = (kCurrentMeasureMaxADCVolts * level) / kCurrentMeasureMaxADCCount;
  // Compensate effect of voltage divider.
  return ((100 + 470) * adc_voltage) / 100;
}

static float GetPowerAmps() {
  return (static_cast<uint32_t>(analogRead(A6)) * kCurrentMeasureMaxADCVolts) / (kCurrentMeasureMaxADCCount * kCurrentMeasureAmplifierGain * kTotalCurrentMeasureResistorOhms);
}

static PowerSource PowerSourceFromPowerVolts(float power_volts) {
  if (power_volts > 5.0f) {
    // TODO: disambiguate between battery and external.
    return PowerSource::kExternalConnector;
  }
  if (Serial) { return PowerSource::kUSB; }
  return PowerSource::kUnknown;
}

PowerSource GetPowerSource() {
  return PowerSourceFromPowerVolts(GetPowerVolts());
}

static float GetMotorsCurrentResistorVolts() {
  return (static_cast<uint32_t>(analogRead(A7)) * kCurrentMeasureMaxADCVolts) / (kCurrentMeasureMaxADCCount * kCurrentMeasureAmplifierGain);
}

static float GetServosCurrentResistorVolts() {
  // TODO: read from correct ADC channel.
  return (static_cast<uint32_t>(analogRead(A7)) * kCurrentMeasureMaxADCVolts) / (kCurrentMeasureMaxADCCount * kCurrentMeasureAmplifierGain);
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

float GetServosAmps() {
  return GetServosCurrentResistorVolts() / kServosCurrentMeasureResistorOhms;
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

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

void PowerOff() {
  // Set pin 5 (PTD7) to switch off the board.
  pinMode(5, OUTPUT);
  digitalWrite(5, 1);
  SleepForNanos(500'000'000);
  digitalWrite(5, 0);
  pinMode(5, INPUT);
}

static ADC power_adc;

static int adc1_accumulator;
static RingBuffer<uint16_t, 64> adc1_samples;

static void power_adc1_isr() {
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

void InitPower() {
  // Start continuous capture of servos current, so it's digitally filtered in the background.
  adc1_accumulator = 0;
  power_adc.adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);
  power_adc.adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);
  power_adc.adc1->setAveraging(32);
  power_adc.adc1->enableInterrupts(&power_adc1_isr);
  power_adc.adc1->startContinuous(A12);
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

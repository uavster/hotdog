#include "kinetis.h"
#include "servo_control.h"
#include "Arduino.h"
#include <algorithm>

// Servo configuration.
#define kServoPWMFrequencyHz 50.0f
#define kServoMsPer180Degrees 6.0f
#define kServoZeroDegreePulseMs 1.5f

#define kTimerTicksPerSecond (16000000 / 32)
#define kPWMPeriodTicks (kTimerTicksPerSecond / kServoPWMFrequencyHz)

#define kHorizontalPitchDegrees 125.0f
#define kHorizontalRollDegrees 90.0f

#define kMinPitchDegrees -60.0f
#define kMaxPitchDegrees 60.0f

#define kMinRollDegrees -45.0f
#define kMaxRollDegrees 45.0f

void InitServos() {
  FTM0_SC = 0;
  // Enable with QUADEN=0, DECAPEN=0, COMBINE=0, CPWMS=0, MSnB=1
  FTM0_QDCTRL = 0;
  FTM0_COMBINE = 0;
  FTM0_MOD = kPWMPeriodTicks - 1;
  FTM0_CNTIN = 0;
  FTM0_CNT = 0;
  FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(5);  // CPWMS=0, CLKS=System clock, PS=Divide clock by 32
  // Set edge-aligned PWM.
  // CH1IE = 0 (interrupt disabled), MS1B:MS0A = 2 and ELS1B:ELS1A = 2 (high-true pulses)
  FTM0_C2SC = FTM_CSC_MSB | FTM_CSC_ELSB;
  FTM0_C3SC = FTM_CSC_MSB | FTM_CSC_ELSB;

  // Set FTM0_CHx function for pins.
  PORTC_PCR3 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
  PORTC_PCR4 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;

  SetHeadPitchDegrees(0);
  SetHeadRollDegrees(0);
}

static float SaturateRoll(float angle_degrees) {
  return std::min(kMaxRollDegrees, std::max(kMinRollDegrees, angle_degrees));
}

static float SaturatePitch(float angle_degrees) {
  return std::min(kMaxPitchDegrees, std::max(kMinPitchDegrees, angle_degrees));
}

void SetHeadPitchDegrees(float angle_degrees) {
  float pulse_width_ms = ((-SaturatePitch(angle_degrees) + kHorizontalPitchDegrees) * kServoMsPer180Degrees) / 180 + kServoZeroDegreePulseMs;
  FTM0_C2V = (pulse_width_ms * kTimerTicksPerSecond) / 1000;
}

void SetHeadRollDegrees(float angle_degrees) {
  float pulse_width_ms = ((SaturateRoll(angle_degrees) + kHorizontalRollDegrees) * kServoMsPer180Degrees) / 180 + kServoZeroDegreePulseMs;
  FTM0_C3V = (pulse_width_ms * kTimerTicksPerSecond) / 1000;
}

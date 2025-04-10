#include "motors.h"
#include "Arduino.h"

// The lower the frequency, the more torque at low speeds, but the less spatial resolution of motor control. 
// The higher the frequency, the lower the duty cycle resolution. 
// In order to have a duty cycle resolution of at least 1%, the frequency should stay under 312.5 Hz.
// The resolution is determined by this formula: duty_cycle=(CnV-CNTIN)*100/period_ticks
#define kPWMFrequencyHz 50
#define kPWMPeriodTicks ((16000000 / 512) / kPWMFrequencyHz)  // 16MHz external crystal / 512 prescaler (FRDIV = 0b100).

void InitMotors() {
  FTM1_SC = 0;
  // Enable with QUADEN=0, DECAPEN=0, COMBINE=0, CPWMS=0, MSnB=1
  FTM1_QDCTRL = 0;
  FTM1_COMBINE = 0;
  FTM1_MOD = kPWMPeriodTicks - 1;
  FTM1_CNTIN = 0;
  FTM1_CNT = 0;
  FTM1_SC = FTM_SC_CLKS(2) | FTM_SC_PS(0);  // CPWMS=0, CLKS=Fixed-Frequency clock, PS=0 (divide clock by 1).
  // Set edge-aligned PWM.
  // CH1IE = 0 (interrupt disabled), MS1B:MS0A = 2 and ELS1B:ELS1A = 2 (high-true pulses)
  FTM1_C0SC = FTM_CSC_MSB | FTM_CSC_ELSB;
  FTM1_C1SC = FTM_CSC_MSB | FTM_CSC_ELSB;

  SetLeftMotorDutyCycle(0);
  SetRightMotorDutyCycle(0);
}

void SetLeftMotorDutyCycle(float s) {
  if (s > 0) {
    pinMode(4, OUTPUT);
    digitalWrite(4, 0);
    // Period=MOD-CNTIN+1 ticks, duty_cycle=(CnV-CNTIN)*100/period_ticks
    FTM1_C1V = (int)((kPWMPeriodTicks - 1) * (65535 * s)) >> 16;
    PORTB_PCR1 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
  } else if (s < 0) {
    pinMode(17, OUTPUT);
    digitalWrite(17, 0);
    // Period=MOD-CNTIN+1 ticks, duty cycle=(CnV-CNTIN)*100/period_ticks
    FTM1_C1V = (int)((kPWMPeriodTicks - 1) * (65535 * -s)) >> 16;
    PORTA_PCR13 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
  } else {
    FTM1_C1V = 0;
    pinMode(17, OUTPUT);
    digitalWrite(17, 0);
    pinMode(4, OUTPUT);
    digitalWrite(4, 0);
  }
}

void SetRightMotorDutyCycle(float s) {
  if (s > 0) {
    pinMode(16, OUTPUT);
    digitalWrite(16, 0);
    // Period=MOD-CNTIN+1 ticks, duty cycle=(CnV-CNTIN)*100/period_ticks
    FTM1_C0V = (int)((kPWMPeriodTicks - 1) * (65535 * s)) >> 16;
    PORTA_PCR12 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
  } else if (s < 0) {
    pinMode(3, OUTPUT);
    digitalWrite(3, 0);
    // Period=MOD-CNTIN+1 ticks, duty cycle=(CnV-CNTIN)*100/period_ticks
    FTM1_C0V = (int)((kPWMPeriodTicks - 1) * (65535 * -s)) >> 16;
    PORTB_PCR0 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;
  } else {
    FTM1_C0V = 0;
    pinMode(3, OUTPUT);
    digitalWrite(3, 0);
    pinMode(16, OUTPUT);
    digitalWrite(16, 0);
  }
}

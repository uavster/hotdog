#include "motors.h"
#include "Arduino.h"

#define kPWMFrequencyHz 2000
#define kPWMPeriodTicks ((16000000 / 32) / kPWMFrequencyHz)

void InitMotors() {
  FTM1_SC = 0;
  // Enable with QUADEN=0, DECAPEN=0, COMBINE=0, CPWMS=0, MSnB=1
  FTM1_QDCTRL = 0;
  FTM1_COMBINE = 0;
  FTM1_MOD = kPWMPeriodTicks - 1;
  FTM1_CNTIN = 0;
  FTM1_CNT = 0;
  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(5);  // CPWMS=0, CLKS=System clock, PS=Divide clock by 32
  // Set edge-aligned PWM.
  // CH1IE = 0 (interrupt disabled), MS1B:MS0A = 2 and ELS1B:ELS1A = 2 (high-true pulses)
  FTM1_C0SC = FTM_CSC_MSB | FTM_CSC_ELSB;
  FTM1_C1SC = FTM_CSC_MSB | FTM_CSC_ELSB;

  SetLeftMotorDutyCycle(0);
  SetRightMotorDutyCycle(0);
}

void SetRightMotorDutyCycle(float s) {
  if (s > 0) {
    pinMode(4, OUTPUT);
    digitalWrite(4, 0);
    // Period=MOD-CNTIN+1 ticks, duty cycle=(CnV-CNTIN)*100/period_ticks
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

void SetLeftMotorDutyCycle(float s) {
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

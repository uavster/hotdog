#define kEventRingBufferCapacity 256

const int ledPin = 13;
uint8_t led_on = 1;

#include "ring_buffer.h"
#include "robot_state.h"
#include "utils.h"
#include "p2p_packet_stream.h"
#include "p2p_byte_stream_arduino.h"
// #include <SPI.h>

// ------------ Event ring buffer ------------
typedef enum {
  kLeftWheelTick,
  kRightWheelTick,
  kLeftWheelForwardCommand,
  kLeftWheelBackwardCommand,
  kRightWheelForwardCommand,
  kRightWheelBackwardCommand,
} EventType;

typedef struct {
  EventType type;
  uint64_t ticks;
} Event;

RingBuffer<Event, kEventRingBufferCapacity> event_buffer;

// ------------ Timer ------------
uint32_t timer0_num_overflows;

bool timer0_is_write_protected() {
  return FTM0_FMS & FTM_FMS_WPEN;
}

void timer0_set_write_protected() {
  // WPEN = 1 (write protection enabled)
  FTM0_FMS |= FTM_FMS_WPEN;
}

void timer0_clear_write_protected() {
  // First, read WPEN
  if (timer0_is_write_protected()) {
    // WPDIS = 1 (write protection disable)
    FTM0_MODE |= FTM_MODE_WPDIS;
  }
}

bool timer1_is_write_protected() {
  return FTM1_FMS & FTM_FMS_WPEN;
}

void timer1_set_write_protected() {
  // WPEN = 1 (write protection enabled)
  FTM1_FMS |= FTM_FMS_WPEN;
}

void timer1_clear_write_protected() {
  // First, read WPEN
  if (timer1_is_write_protected()) {
    // WPDIS = 1 (write protection disable)
    FTM1_MODE |= FTM_MODE_WPDIS;
  }
}

uint32_t timer_ticks() {
  NVIC_DISABLE_IRQ(IRQ_FTM0);
  uint32_t cur_time = (timer0_num_overflows << 16) | (FTM0_CNT & 0xffff);
  NVIC_DISABLE_IRQ(IRQ_FTM0);
  return cur_time;
}

void init_motor_control() {
  #define kPWMFrequencyHz   20
  #define kPWMPeriodTicks   ((16000000/32)/kPWMFrequencyHz)
  FTM1_SC = 0;
  FTM1_CNT = 0;
  FTM1_MOD = kPWMPeriodTicks - 1;
  FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(5); // CPWMS=0, CLKS=System clock, PS=Divide clock by 32
  // Set edge-aligned PWM 
  // Enable with QUADEN=0, DECAPEN=0, COMBINE=0, CPWMS=0, MSnB=1
  FTM1_QDCTRL = 0;
  FTM1_COMBINE = 0;
  // CH1IE = 0 (interrupt disabled), MS1B:MS0A = 2 and ELS1B:ELS1A = 2 (high-true pulses)
  FTM1_C0SC = FTM_CSC_MSB | FTM_CSC_ELSB;

  FTM1_CNTIN = 0;

  set_duty_cycle_left(0);
  set_duty_cycle_right(0);
}

void set_duty_cycle_right(float s) {
  if (s > 0) {
    pinMode(16, OUTPUT);
    digitalWrite(16, 0);
    // Period=MOD-CNTIN+1 ticks, duty cycle=(CnV-CNTIN)*100/period_ticks
    FTM1_C0V = (int)((kPWMPeriodTicks - 1) * (65535 * s)) >> 16;
    PORTA_PCR12 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;  
    // Disable the IRQ while writing to the event buffer to avoid a race.
    NVIC_DISABLE_IRQ(IRQ_FTM0);
    event_buffer.Write(Event{.type = kRightWheelForwardCommand, .ticks = timer_ticks()});
    NVIC_ENABLE_IRQ(IRQ_FTM0);
  } else if (s < 0) {
    pinMode(3, OUTPUT);
    digitalWrite(3, 0);
    // Period=MOD-CNTIN+1 ticks, duty cycle=(CnV-CNTIN)*100/period_ticks
    FTM1_C0V = (int)((kPWMPeriodTicks - 1) * (65535 * -s)) >> 16;
    PORTB_PCR0 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;  
    // Disable the IRQ while writing to the event buffer to avoid a race.
    NVIC_DISABLE_IRQ(IRQ_FTM0);
    event_buffer.Write(Event{.type = kRightWheelBackwardCommand, .ticks = timer_ticks()});
    NVIC_ENABLE_IRQ(IRQ_FTM0);
  } else {
    FTM1_C0V = 0;
    pinMode(3, OUTPUT);
    digitalWrite(3, 0);
    pinMode(16, OUTPUT);
    digitalWrite(16, 0);
  }
}

void set_duty_cycle_left(float s) {
  if (s > 0) {
    pinMode(4, OUTPUT);
    digitalWrite(4, 0);
    // Period=MOD-CNTIN+1 ticks, duty cycle=(CnV-CNTIN)*100/period_ticks
    FTM1_C1V = (int)((kPWMPeriodTicks - 1) * (65535 * s)) >> 16;
    PORTB_PCR1 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;  
    // Disable the IRQ while writing to the event buffer to avoid a race.
    NVIC_DISABLE_IRQ(IRQ_FTM0);
    event_buffer.Write(Event{.type = kLeftWheelForwardCommand, .ticks = timer_ticks()});
    NVIC_ENABLE_IRQ(IRQ_FTM0);
  } else if (s < 0) {
    pinMode(17, OUTPUT);
    digitalWrite(17, 0);
    // Period=MOD-CNTIN+1 ticks, duty cycle=(CnV-CNTIN)*100/period_ticks
    FTM1_C1V = (int)((kPWMPeriodTicks - 1) * (65535 * -s)) >> 16;
    PORTA_PCR13 = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE;  
    // Disable the IRQ while writing to the event buffer to avoid a race.
    NVIC_DISABLE_IRQ(IRQ_FTM0);
    event_buffer.Write(Event{.type = kLeftWheelBackwardCommand, .ticks = timer_ticks()});
    NVIC_ENABLE_IRQ(IRQ_FTM0);
  } else {
    FTM1_C1V = 0;
    pinMode(17, OUTPUT);
    digitalWrite(17, 0);
    pinMode(4, OUTPUT);
    digitalWrite(4, 0);
  }
}

P2PByteStreamArduino byte_stream(&Serial1);
P2PPacketInputStream<3, kLittleEndian> p2p_input_stream(&byte_stream);
P2PPacketOutputStream<3, kLittleEndian> p2p_output_stream(&byte_stream);

void setup() {
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);
  Serial1.begin(115200);
  Serial1.attachRts(2);

  timer0_num_overflows = 0;

/*  timer0_clear_write_protected();
  // FTMEN = 1 (enable all registers)
  FTM0_MODE |= 0x1;*/
/*  
 *   This is set up by the teensy boot up routine.
  // External oscillator is 16 MHz in Teensy 3.2
  // FRDIV = 100b (clock/512), IREFS = 0 (get fixed frequency clock from external+prescalers)
  MCG_C1 = 0x20;
  // RANGE0 = 01b (high frequency range for crystal oscillator), HGO0 = 0 (XTAL low-power operation), EREFS0 = 1 (external reference is oscillator)
  MCG_C2 = 0x14;  // Clock does not start well with HGO0 = 1
  // OSCSEL = 0 (system oscillator)
  MCG_C7 = 0;
*/

  FTM0_SC = 0;
  // Total prescaler is 512 because RANGE0!=0 and FRDIV=100b
  // This results in 16e6/512=31.25kHz, which is within the range
  // specified in the FRDIV documentation.  
  // DECAPEN = 0, COMBINE = 0
  FTM0_COMBINE = 0;
  // TOIE = 1 (timer overflow interrupt enabled), CLKS = 2 (fixed frequency clock), PS = 0 (divide clock by 1)
  // CPWMS = 0 (up counting mode)
  FTM0_SC = FTM_SC_TOIE | FTM_SC_CLKS(2);
  // QUADEN = 0 (quadrature decoder mode disabled)
  FTM0_QDCTRL = 0;
  // Configure timer 0's channels 0 and 1 for input capture on rising edge
  // Table 36-67
  // CH0IE = 1 (interrupt enabled), MS0B:MS0A = 0 and ELS0B:ELS0A = 1 (capture on rising edge)
  FTM0_C0SC = FTM_CSC_CHIE | FTM_CSC_ELSA;
  PORTC_PCR1 = PORT_PCR_MUX(4);
  FTM0_C1SC = FTM_CSC_CHIE | FTM_CSC_ELSA;
  PORTC_PCR2 = PORT_PCR_MUX(4);
  // INIT = 0 (count initial value is 0)
  FTM0_CNTIN = 0;
  // Set counter to CNTIN value
  FTM0_CNT = 0;
  // Set counter modulo
  FTM0_MOD = 0xffff;
  // NUMTOF = 0 (TOF set at each counter overflow)
  FTM0_CONF = 0;

  FTM0_FILTER = FTM_FILTER_CH0FVAL(15) | FTM_FILTER_CH1FVAL(15);

  NVIC_ENABLE_IRQ(IRQ_FTM0);

  init_motor_control();  

//  timer0_set_write_protected();
/*
  PORTA_PCR12 = PORT_PCR_MUX(1);
  SIM_SOPT4 &= SIM_SOPT4_FTM1CH0SRC(0);
  SIM_SCGC5 |= 1 << 10;
  SIM_SCGC6 |= 1 << 25;
*/  
//  timer1_clear_write_protected();
  // FTMEN = 1 (enable all registers)
//  FTM1_MODE |= FTM_MODE_FTMEN;
  
  // Pin 1 controls direction of the right wheel.
//  pinMode(17, OUTPUT);
//  digitalWrite(17, 1);

  
//  timer1_set_write_protected();
//  analogWriteFrequency(16, 100);
//  analogWrite(16, 64);

/*
  // PORTD_PCR1 = PORT_PCR_MUX(2);
  // PORTD_PCR2 = PORT_PCR_MUX(2);
  // PORTD_PCR3 = PORT_PCR_MUX(2);
  SPI.setSCK(14);
  SPI.setMISO(8);
  SPI.setMOSI(7);
  SPI.begin();  
  pinMode(2, OUTPUT);   // SPI CS
  digitalWrite(2, HIGH);
  */
}

void ftm0_isr(void) {
  if (FTM0_SC & FTM_SC_TOF) {
    // The timer 0 counter overflowed
    timer0_num_overflows++;
    // Reset overflow flag (read SC and write 0 to TOF)
    FTM0_SC &= ~0x80;
    return;
  }
  if (FTM0_C0SC & FTM_CSC_CHF) {
    // Toggle LED
    digitalWrite(ledPin, led_on);
    led_on = (led_on + 1) & 1;
    // We have exclusive access to the event buffer while in the ISR.
    event_buffer.Write(Event{.type = kLeftWheelTick, .ticks = (timer0_num_overflows << 16) | (FTM0_C0V & 0xffff)});
    FTM0_C0SC &= ~FTM_CSC_CHF;
    return;
  }
  if (FTM0_C1SC & FTM_CSC_CHF) {
    // Toggle LED
    digitalWrite(ledPin, led_on);
    led_on = (led_on + 1) & 1;
    // We have exclusive access to the event buffer while in the ISR.
    event_buffer.Write(Event{.type = kRightWheelTick, .ticks = (timer0_num_overflows << 16) | (FTM0_C1V & 0xffff)});
    FTM0_C1SC &= ~FTM_CSC_CHF;
    return;
  }
}

//char format_buffer[32];

enum { INIT, CIRCLING_LEFT, CIRCLING_RIGHT, WAIT_FOR_INPUT, SEND_TRAJECTORY, DONE } state = INIT;
int64_t cycles = 0;

#define kWaitCycles 100
#define kInnerDutyCycle 0.25

RobotState robot_state;

typedef struct {
  Point center;
  float angle;
} TrajectoryPoint;

#define kMaxTrajectoryPoints  3000
int cur_trajectory_point = 0;
TrajectoryPoint trajectory[kMaxTrajectoryPoints];
int send_index = 0;
char tmp[128];

// char a = '0';

StatusOr<P2PMutablePacketView> current_packet_view(kUnavailableError);

void loop() {
  if (current_packet_view.ok()) {
    while (Serial.available() > 0) {  
      char c = Serial.read();
      if (c == 0x13) {
        p2p_output_stream.Commit();
        current_packet_view = StatusOr<P2PMutablePacketView>(kUnavailableError);
      } else {
        current_packet_view->content()[current_packet_view->length()] = c;
        ++current_packet_view->length();
      }
    }
  } else {
    current_packet_view = p2p_output_stream.NewPacket();
    if (current_packet_view.ok()) {
      current_packet_view->length() = 0;
    }
  }

  while(p2p_input_stream.OldestPacket().ok()) {
  Serial.println("heelo");
    // Serial.write(p2p_input_stream.OldestPacket()->content(), p2p_input_stream.OldestPacket()->length());
    for (int i = 0; i < p2p_input_stream.OldestPacket()->length(); ++i) {
      Serial.printf("0x%x ", p2p_input_stream.OldestPacket()->content()[i]);
    }
    Serial.println();
    p2p_input_stream.Consume();
  }

  p2p_input_stream.Run();
  p2p_output_stream.Run();

  return;
/*
 if (SPI.pinIsChipSelect(2)) { Serial.println("2 is CS"); }
  else { Serial.println("2 is NOT CS");}
  if (SPI.pinIsSCK(14)) { Serial.println("14 is SCK"); }
  else { Serial.println("14 is NOT SCK");}
  if (SPI.pinIsMISO(8)) { Serial.println("8 is MISO"); }
  else { Serial.println("8 is NOT MISO");}
  if (SPI.pinIsMOSI(7)) { Serial.println("7 is MOSI"); }
  else { Serial.println("7 is NOT MOSI");}

    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));
    Serial.println("Enabling CS");
    digitalWrite(2, LOW); // select device
    delay(5000);
    Serial.println("Transferring");
    
    char buffer[1000];
    for (int i = 0; i < (int)sizeof(buffer); ++i) buffer[i] = 0xaa;
    for (int i = 0; i < 100; ++i) {
    SPI.transfer(buffer, sizeof(buffer));
    }
    Serial.println("Disabling CS");
    digitalWrite(2, HIGH);
    SPI.endTransaction();
    delay(5000);
    return;
*/
/*
    Serial1.write(a);
    a++;
    if (a > 'z') { a = '0'; }

  int k = Serial1.read();
  if (k >= 0) {
    Serial.println(k);
  }
  return;
  */

/*  if (event_buffer.NumEvents() > 0) {
    const Event event = event_buffer.Read();
    Uint64ToString(event.ticks, format_buffer);
    switch(event.type) {
      case kLeftWheelTick:
        Serial.print("L: ");
        break;
      case kRightWheelTick:
        Serial.print("R: ");
        break;
    }
    Serial.println(format_buffer);
  }
*/

/*  static float t = 0; 
  float left_command = -sin(2*3.14159f*0.1*t);
  float right_command = sin(2*3.14159f*0.1*t); 
  set_duty_cycle_left(left_command);
  set_duty_cycle_right(right_command);
  t += 0.00005f;
*/
  
  switch(state) {
    case INIT:
      set_duty_cycle_left(kInnerDutyCycle);
      set_duty_cycle_right(1.0);
      cycles = 0;
      state = CIRCLING_LEFT;
      break;
    case CIRCLING_LEFT:
      if (cycles > kWaitCycles) { 
        set_duty_cycle_left(1.0);
        set_duty_cycle_right(kInnerDutyCycle);
        cycles = 0;
        state = CIRCLING_RIGHT;
      }
      break;
    case CIRCLING_RIGHT:
      if (cycles > kWaitCycles) { 
        set_duty_cycle_left(0);
        set_duty_cycle_right(0);
        cycles = 0;
        state = WAIT_FOR_INPUT;
      }
      break;
    case WAIT_FOR_INPUT:
      if (Serial.available() > 0 && Serial.read() == 'n') {
        state = SEND_TRAJECTORY;
        send_index = 0;
        sprintf(tmp, "Sending trajectory with %d points...", cur_trajectory_point);
        Serial.println(tmp);
      }
      break;
    case SEND_TRAJECTORY:
      sprintf(tmp, "%f,%f,%f", trajectory[send_index].center.x, trajectory[send_index].center.y, trajectory[send_index].angle);
      Serial.println(tmp);
      ++send_index;
      if (send_index >= cur_trajectory_point) {
        state = DONE;
      }
      break;
    case DONE:
      break;
  }

  if (event_buffer.Size() > 0) {
    int left_ticks = 0;
    int right_ticks = 0;
    while (event_buffer.Size() > 0) {
      ++cycles;
      // Disable the IRQ while reading from the event buffer to avoid a race.
      NVIC_DISABLE_IRQ(IRQ_FTM0);
      const Event event = event_buffer.Read();
      NVIC_ENABLE_IRQ(IRQ_FTM0);
      switch(event.type) {
        case kLeftWheelTick:
          ++left_ticks;
          robot_state.NotifyWheelTicks(left_ticks, right_ticks);
          break;
        case kRightWheelTick:
          ++right_ticks;
          robot_state.NotifyWheelTicks(left_ticks, right_ticks);
          break;
        case kLeftWheelForwardCommand:
          robot_state.NotifyLeftWheelDirection(false);
          break;
        case kLeftWheelBackwardCommand:
          robot_state.NotifyLeftWheelDirection(true);
          break;
        case kRightWheelForwardCommand:
          robot_state.NotifyRightWheelDirection(false);
          break;
        case kRightWheelBackwardCommand:
          robot_state.NotifyRightWheelDirection(true);
          break;
      }
    }
    if ((state == INIT || state == CIRCLING_LEFT || state == CIRCLING_RIGHT) && cur_trajectory_point < kMaxTrajectoryPoints) {
      trajectory[cur_trajectory_point].center = robot_state.Center();
      trajectory[cur_trajectory_point].angle = robot_state.Angle();
      ++cur_trajectory_point;
    }
  }

//  set_duty_cycle_left(1.0f);
//  set_duty_cycle_right(1.0f);
}

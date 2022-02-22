#define EVENT_RING_BUFFER_SIZE  256

const int ledPin = 13;
uint8_t led_on = 1;

// ------------ Event ring buffer ------------
typedef enum {
  kLeftWheelTick,
  kRightWheelTick
} EventType;

typedef struct {
  EventType type;
  uint64_t ticks;
} Event;

class EventRingBuffer {
  private:
    Event events_[EVENT_RING_BUFFER_SIZE];
    int read_index_;
    int write_index_;
    volatile int num_events_;

  public:
    EventRingBuffer() : read_index_(0), write_index_(0) {
    }

    inline void IncReadIndex() {
      read_index_ = (read_index_ + 1) % EVENT_RING_BUFFER_SIZE;
    }

    inline void IncWriteIndex() {
      write_index_ = (write_index_ + 1) % EVENT_RING_BUFFER_SIZE;
    }

    // Returns the number of events in the buffer.
    inline int NumEvents() {
      return num_events_;
    }

    // Returns the oldest value in the buffer. This function blocks until a
    // an event becomes available. It should not be called from an ISR.
    Event Read() {
      while (NumEvents() == 0) {}
      NVIC_DISABLE_IRQ(IRQ_FTM0);
      Event event = events_[read_index_];
      IncReadIndex();
      num_events_--;
      NVIC_ENABLE_IRQ(IRQ_FTM0);
      return event;
    }

    // Writes a new value in the buffer. The caller must guarantee that no
    // interrupts will alter the buffer indices while this function runs.
    void Write(const Event &event) {
      events_[write_index_] = event;
      IncWriteIndex();
      num_events_++;
      if (write_index_ == read_index_) {
        // Claim oldest unread slot for writing
        IncReadIndex();
      }
    }
};

EventRingBuffer event_buffer;

// ------------ State ------------
#define kRobotDistanceBetweenTireCenters (0.13 - 0.025)
#define kWheelRadius 0.0325
#define kRadiansPerWheelTick (M_PI/10)   // 20 ticks per wheel encoder

class Point {
  public:
    double x;
    double y;

    Point(): x(0), y(0) {}
};

class RobotState {
  private:
    uint64_t left_wheel_ticks_;
    uint64_t right_wheel_ticks_;
    double angle_;
    Point center_;

  public:
    RobotState() : left_wheel_ticks_(0), right_wheel_ticks_(0), angle_(0.0) {}
    
    void NotifyWheelTicks(int left_ticks_inc, int right_ticks_inc) {
      left_wheel_ticks_ += left_ticks_inc;
      right_wheel_ticks_ += right_ticks_inc;
      double distance_inc = (kRadiansPerWheelTick * kWheelRadius * (left_ticks_inc + right_ticks_inc)) / 2;
      angle_ = ((kRadiansPerWheelTick * kWheelRadius) * (right_wheel_ticks_ - left_wheel_ticks_)) / kRobotDistanceBetweenTireCenters;
      center_.x += distance_inc * cos(angle_);
      center_.y += distance_inc * sin(angle_);
    }

    const Point &Center() {
      return center_;
    }

    const double Angle() {
      return angle_;
    }
};

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
}

void set_duty_cycle_right(float s) {
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

void set_duty_cycle_left(float s) {
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

void setup() {
  pinMode(ledPin, OUTPUT);

  Serial.begin(38400);

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
    event_buffer.Write(Event{.type = kLeftWheelTick, .ticks = (timer0_num_overflows << 16) | (FTM0_C0V & 0xffff)});
    FTM0_C0SC &= ~FTM_CSC_CHF;
    return;
  }
  if (FTM0_C1SC & FTM_CSC_CHF) {
    // Toggle LED
    digitalWrite(ledPin, led_on);
    led_on = (led_on + 1) & 1;
    event_buffer.Write(Event{.type = kRightWheelTick, .ticks = (timer0_num_overflows << 16) | (FTM0_C1V & 0xffff)});
    FTM0_C1SC &= ~FTM_CSC_CHF;
    return;
  }
}

void Uint64ToString(uint64_t number, char *str) {
  int str_length = 1;
  if (number >= 10) {
    str_length = log10(number) + 1;
  }
  for (int i = str_length - 1; i >= 0; --i) {
    str[i] = '0' + (number % 10);
    number /= 10;
  }
  str[str_length] = '\0';
}

//char format_buffer[32];

//enum { CIRCLING_LEFT, CIRCLING_RIGHT } state = CIRCLING_LEFT;

//int64_t cycles = 0;

//#define kWaitCycles 5000000L
//#define kInnerDutyCycle 0.25

RobotState robot_state;

void loop() {
/*  if (event_buffer.NumEvents() > 0) {
    const Event &event = event_buffer.Read();
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
  
/*  switch(state) {
    case CIRCLING_LEFT:
      if (cycles > kWaitCycles) { 
        set_duty_cycle_left(kInnerDutyCycle);
        set_duty_cycle_right(1.0);
        cycles = 0;
        state = CIRCLING_RIGHT;
      }
      break;
    case CIRCLING_RIGHT:
      if (cycles > kWaitCycles) { 
        set_duty_cycle_left(1.0);
        set_duty_cycle_right(kInnerDutyCycle);
        cycles = 0;
        state = CIRCLING_LEFT;
      }
      break;
  }
  ++cycles;
*/

  if (event_buffer.NumEvents() > 0) {
    int left_ticks = 0;
    int right_ticks = 0;
    while (event_buffer.NumEvents() > 0) {
      const Event &event = event_buffer.Read();
      switch(event.type) {
        case kLeftWheelTick:
          ++left_ticks;
          break;
        case kRightWheelTick:
          ++right_ticks;
          break;
      }
    }
    robot_state.NotifyWheelTicks(left_ticks, right_ticks);
    char text_buffer[32];
    sprintf(text_buffer, "(%.02f, %.02f) %.02f", robot_state.Center().x, robot_state.Center().y, (robot_state.Angle() * 180.0) / M_PI);
    Serial.println(text_buffer);
  }
}

#define EVENT_RING_BUFFER_SIZE  256

const int ledPin = 13;
uint8_t led_on = 1;

#define TIMER0_WPEN_MASK      0x40
#define TIMER0_WPDIS_MASK     0x4
#define TIMER0_OVERFLOW_MASK  0x80

// ------------ Event ring buffer ------------
typedef struct {
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
        Serial.print("gotcha!\n");
      }
    }
};

EventRingBuffer event_buffer_left;
EventRingBuffer event_buffer_right;

// ------------ Timer ------------
uint32_t timer0_num_overflows;

bool timer0_is_write_protected() {
  return FTM0_FMS & TIMER0_WPEN_MASK;
}

void timer0_set_write_protected() {
  // WPEN = 1 (write protection enabled)
  FTM0_FMS |= TIMER0_WPEN_MASK;
}

void timer0_clear_write_protected() {
  // First, read WPEN
  if (timer0_is_write_protected()) {
    // WPDIS = 1 (write protection disable)
    FTM0_MODE |= TIMER0_WPDIS_MASK;
  }
}

void setup() {
  pinMode(ledPin, OUTPUT);

  Serial.begin(38400);

  timer0_num_overflows = 0;

  timer0_clear_write_protected();
  // FTMEN = 1 (enable all registers)
  FTM0_MODE |= 0x1;
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

  NVIC_ENABLE_IRQ(IRQ_FTM0);

  timer0_set_write_protected();
}

void ftm0_isr(void) {
  if (FTM0_SC & TIMER0_OVERFLOW_MASK) {
    // The timer 0 counter overflowed
    timer0_num_overflows++;
    // Reset overflow flag (read SC and write 0 to TOF)
    FTM0_SC &= ~0x80;
  }
  if (FTM0_C0SC & FTM_CSC_CHF) {
    // Toggle LED
    digitalWrite(ledPin, led_on);
    led_on = (led_on + 1) & 1;
    FTM0_C0SC &= ~FTM_CSC_CHF;
    event_buffer_left.Write(Event{.ticks = (timer0_num_overflows << 16) | (FTM0_C0V & 0xffff)});
  }
  if (FTM0_C1SC & FTM_CSC_CHF) {
    // Toggle LED
    digitalWrite(ledPin, led_on);
    led_on = (led_on + 1) & 1;
    FTM0_C1SC &= ~FTM_CSC_CHF;
    event_buffer_right.Write(Event{.ticks = (timer0_num_overflows << 16) | (FTM0_C0V & 0xffff)});
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

char format_buffer[32];

void loop() {
  if (event_buffer_left.NumEvents() > 0) {
    Uint64ToString(event_buffer_left.Read().ticks, format_buffer);
    Serial.print("L: ");
    Serial.println(format_buffer);
  }
  if (event_buffer_right.NumEvents() > 0) {
    Uint64ToString(event_buffer_right.Read().ticks, format_buffer);
    Serial.print("R: ");
    Serial.println(format_buffer);
  }
}

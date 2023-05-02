#define kEventRingBufferCapacity 256

const int ledPin = 13;
uint8_t led_on = 1;

#include "robot_state.h"
#include "utils.h"
#include "p2p_byte_stream_arduino.h"
#include "p2p_packet_stream.h"
#include "motor_control.h"
#include "timer.h"
#include "encoders.h"

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

void LeftEncoderIsr(uint32_t timer_ticks) {
  // Toggle LED
  digitalWrite(ledPin, led_on);
  led_on = (led_on + 1) & 1;
  // We have exclusive access to the event buffer while in the ISR.
  event_buffer.Write(Event{ .type = kLeftWheelTick, .ticks = timer_ticks });
}

void RightEncoderIsr(uint32_t timer_ticks) {
    // Toggle LED
    digitalWrite(ledPin, led_on);
    led_on = (led_on + 1) & 1;
    // We have exclusive access to the event buffer while in the ISR.
    event_buffer.Write(Event{ .type = kRightWheelTick, .ticks = timer_ticks });
}

P2PByteStreamArduino byte_stream(&Serial1);
P2PPacketInputStream<16, kLittleEndian> p2p_input_stream(&byte_stream);
P2PPacketOutputStream<16, kLittleEndian> p2p_output_stream(&byte_stream);

TimerNanosType last_msg_time_ns = 0;

// uint8_t rx_buffer[256];

int last_received_packet_value[P2PPriority::kNumLevels];
int received_packets[P2PPriority::kNumLevels];
int sent_packets[P2PPriority::kNumLevels];
int last_received_packets[P2PPriority::kNumLevels];
int last_sent_packets[P2PPriority::kNumLevels];
int lost_packets[P2PPriority::kNumLevels];

void setup() {
  // Open serial port before anything else, as it enables showing logs and asserts in the console.
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 1);

  InitTimer();

  InitEncoders();
  SetEncoderIsrs(&LeftEncoderIsr, &RightEncoderIsr);

  // Serial1.addMemoryForRead(rx_buffer, 256);
  Serial1.begin(1000000, SERIAL_8N1);

  InitMotors();

  Serial.println("Started.");  

  last_msg_time_ns = GetTimerNanoseconds();

  for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
    last_received_packet_value[i] = -1;
    received_packets[i] = 0;
    sent_packets[i] = 0;
    last_received_packets[i] = 0;
    last_sent_packets[i] = 0;
    lost_packets[i] = 0;
  }

  // UART0_S2 |= UART_S2_LBKDE;
  // Clear FE.
  // noInterrupts();
  // uint8_t uart_s1 = UART0_S1;  
  // ++uart_s1;
  // uint8_t uart_d = UART0_D;
  // ++uart_d;
  // interrupts();
  // Serial.println("After clear");
  // Serial.printf("c1=%x c2=%x c3=%x c4=%x c5=%x\n", UART0_C1, UART0_C2, UART0_C3, UART0_C4, UART0_C5);
  // Serial.printf("s1=%x s2=%x\n", UART0_S1, UART0_S2);

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

//char format_buffer[32];

enum { INIT,
       CIRCLING_LEFT,
       CIRCLING_RIGHT,
       WAIT_FOR_INPUT,
       SEND_TRAJECTORY,
       DONE } state = INIT;
int64_t cycles = 0;

#define kWaitCycles 100
#define kInnerDutyCycle 0.25

RobotState robot_state;

typedef struct {
  Point center;
  float angle;
} TrajectoryPoint;

#define kMaxTrajectoryPoints 3000
int cur_trajectory_point = 0;
TrajectoryPoint trajectory[kMaxTrajectoryPoints];
int send_index = 0;
char tmp[128];

// char a = '0';

StatusOr<P2PMutablePacketView> current_packet_view(kUnavailableError);

// uint32_t last_packet_send_time = 0;

TimerNanosType last_sent_packet_nanos = 0;

void loop() {
  static uint8_t count = 0;
  TimerNanosType now_ns = GetTimerNanoseconds();

  // Working values:
  // 9600 bps -> 230
  // 115200 bps -> 18
  // 1000000 bps -> 2

  int len = 10;
  if (now_ns - last_sent_packet_nanos >= 9000000) {
    last_sent_packet_nanos = now_ns;
    P2PPriority priority = P2PPriority::Level::kHigh;
    current_packet_view = p2p_output_stream.NewPacket(priority);
    if (current_packet_view.ok()) {
      for (int i = 0; i < len; ++i) {
        reinterpret_cast<uint8_t *>(current_packet_view->content())[i] = 0;
      }
      *reinterpret_cast<uint8_t *>(current_packet_view->content()) = count++;
      current_packet_view->length() = len; //sizeof(uint8_t);
      ASSERT(p2p_output_stream.Commit(priority)); 

      ++sent_packets[priority];
      // ++len;
      // if (len == 0xa9) { len = 1; }
    }
  }

  len = 0xa8;
  P2PPriority priority2 = P2PPriority::Level::kLow;
  current_packet_view = p2p_output_stream.NewPacket(priority2);
  if (current_packet_view.ok()) {
    for (int i = 0; i < len; ++i) {
      reinterpret_cast<uint8_t *>(current_packet_view->content())[i] = 0;
    }
    *reinterpret_cast<uint8_t *>(current_packet_view->content()) = count++;
    current_packet_view->length() = len; //sizeof(uint8_t);
    ASSERT(p2p_output_stream.Commit(priority2)); 
    ++sent_packets[priority2];
    // ++len;
    // if (len == 0xa9) { len = 1; }
  }

  if (now_ns - last_msg_time_ns >= 1e9) {
    Serial.print("tx:");
    for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
      Serial.printf("%d ", sent_packets[i] - last_sent_packets[i]);
    }
    Serial.print(", rx:");
    for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
      Serial.printf("%d ", received_packets[i] - last_received_packets[i]);
    }
    Serial.print(", lost:");
    for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
      Serial.printf("%d ", lost_packets[i]);
    }
    Serial.println("");
    for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
      last_sent_packets[i] = sent_packets[i];
      last_received_packets[i] = received_packets[i];
    }
    last_msg_time_ns = now_ns;
  }
  
  /*
  priority = P2PPriority::Level::kMedium;
  if (current_packet_view.ok()) {
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 0xa) {
        p2p_output_stream.Commit(priority);
        current_packet_view = StatusOr<P2PMutablePacketView>(kUnavailableError);
      } else {
        current_packet_view->content()[current_packet_view->length()] = c;
        ++current_packet_view->length();
      }
    }
  } else {
    current_packet_view = p2p_output_stream.NewPacket(priority);
    if (current_packet_view.ok()) {
      current_packet_view->length() = 0;
    }
  }*/

  while (p2p_input_stream.OldestPacket().ok()) {
    // Serial.write(p2p_input_stream.OldestPacket()->content(), p2p_input_stream.OldestPacket()->length());
    // Serial.print("p: ");
    // for (int i = 0; i < p2p_input_stream.OldestPacket()->length(); ++i) {
    //   Serial.printf("%x ", p2p_input_stream.OldestPacket()->content()[i]);
    // }
    // Serial.println();
    
    P2PPriority priority = p2p_input_stream.OldestPacket()->priority();
    ++received_packets[priority];
    if (last_received_packet_value[priority] >= 0) {
      int diff = p2p_input_stream.OldestPacket()->content()[0] - last_received_packet_value[priority];
      if (diff > 0) lost_packets[priority] += diff - 1;
      else lost_packets[priority] += 256 + diff - 1;
    }
    last_received_packet_value[priority] = p2p_input_stream.OldestPacket()->content()[0];
    p2p_input_stream.Consume();
  }

  p2p_input_stream.Run();
  p2p_output_stream.Run(GetTimerNanoseconds());

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

  switch (state) {
    case INIT:
      SetLeftMotorDutyCycle(kInnerDutyCycle);
      SetRightMotorDutyCycle(1.0);
      // Disable the IRQ while writing to the event buffer to avoid a race.
      NVIC_DISABLE_IRQ(IRQ_FTM0);
      event_buffer.Write(Event{ .type = kInnerDutyCycle >= 0 ? kLeftWheelForwardCommand : kLeftWheelBackwardCommand, .ticks = GetTimerTicks() });
      event_buffer.Write(Event{ .type = kRightWheelForwardCommand, .ticks = GetTimerTicks() });
      NVIC_ENABLE_IRQ(IRQ_FTM0);
      cycles = 0;
      state = CIRCLING_LEFT;
      break;
    case CIRCLING_LEFT:
      if (cycles > kWaitCycles) {
        SetLeftMotorDutyCycle(1.0);
        SetRightMotorDutyCycle(kInnerDutyCycle);
        // Disable the IRQ while writing to the event buffer to avoid a race.
        NVIC_DISABLE_IRQ(IRQ_FTM0);
        event_buffer.Write(Event{ .type = kLeftWheelForwardCommand, .ticks = GetTimerTicks() });
        event_buffer.Write(Event{ .type = kInnerDutyCycle >= 0 ? kRightWheelForwardCommand : kRightWheelBackwardCommand, .ticks = GetTimerTicks() });
        NVIC_ENABLE_IRQ(IRQ_FTM0);
        cycles = 0;
        state = CIRCLING_RIGHT;
      }
      break;
    case CIRCLING_RIGHT:
      if (cycles > kWaitCycles) {
        SetLeftMotorDutyCycle(0);
        SetRightMotorDutyCycle(0);
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
      switch (event.type) {
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

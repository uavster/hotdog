#define kEventRingBufferCapacity 256

#include "robot_state.h"
#include "utils.h"
#include "p2p_byte_stream_arduino.h"
#include "p2p_packet_stream.h"
#include "motor_control.h"
#include "servo_control.h"
#include "timer.h"
#include "encoders.h"
#include "timer_arduino.h"
#include "guid_factory.h"
#include "logger.h"
#include "body_imu.h"

Logger logger;

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
  // We have exclusive access to the event buffer while in the ISR.
  event_buffer.Write(Event{ .type = kLeftWheelTick, .ticks = timer_ticks });
}

void RightEncoderIsr(uint32_t timer_ticks) {
  // We have exclusive access to the event buffer while in the ISR.
  event_buffer.Write(Event{ .type = kRightWheelTick, .ticks = timer_ticks });
}

P2PByteStreamArduino byte_stream(&Serial1);
TimerArduino timer;
GUIDFactory guid_factory;
// P2PPacketInputStream<8, kLittleEndian> p2p_input_stream(&byte_stream, &timer);
// P2PPacketOutputStream<1, kLittleEndian> p2p_output_stream(&byte_stream, &timer);
P2PPacketStream<8, 1, kLittleEndian> p2p_stream(&byte_stream, &timer, guid_factory);

TimerNanosType last_msg_time_ns = 0;

// uint8_t rx_buffer[256];

int last_received_packet_value[P2PPriority::kNumLevels];
int received_packets[P2PPriority::kNumLevels];
int sent_packets[P2PPriority::kNumLevels];
int last_received_packets[P2PPriority::kNumLevels];
int last_sent_packets[P2PPriority::kNumLevels];
int lost_packets[P2PPriority::kNumLevels];

BodyIMU body_imu;

void setup() {
  // Open serial port before anything else, as it enables showing logs and asserts in the console.
  Serial.begin(115200);

  *logger.base_logger() = SetLogger(&logger);

  InitTimer();

  // Serial starts working after a second or so. Wait, so we don't miss any log.
  while(GetTimerNanoseconds() < 1000000000ULL) {}

  InitEncoders();
  SetEncoderIsrs(&LeftEncoderIsr, &RightEncoderIsr);

  body_imu.Init();

  // Serial1.addMemoryForRead(rx_buffer, 256);
  Serial1.begin(1000000, SERIAL_8N1);

  InitMotors();
  InitServos();

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
       SLOW_DOWN,
       WAIT_FOR_INPUT,
       SEND_TRAJECTORY,
       DONE } state = INIT;
uint64_t last_timestamp_ns = 0;

#define kWaitSeconds 3
#define kInnerDutyCycle 0.75

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

uint64_t last_ns = 0;
uint64_t last_head_ns = 0;

enum {
  LOOKING_FORWARD,
  LOOKING_DOWN,
  LOOKING_UP,
  TILTING_HEAD,
  HEAD_STOP
} head_state = LOOKING_FORWARD;

void loop() {
  // uint64_t now_ns = GetTimerNanoseconds();
  // if (now_ns - last_ns >= 1000000000) {
  //   char tmp[64];
  //   Uint64ToString(now_ns, tmp);
  //   Serial.println(tmp);
  //   last_ns = now_ns;
  // }

  // return;

  if (GetTimerNanoseconds() < 15000000000) {
    last_head_ns = GetTimerNanoseconds();
    return;
  }
/*  TimerNanosType now_ns = GetTimerNanoseconds();

  // Working values:
  // 9600 bps -> 230
  // 115200 bps -> 18
  // 1000000 bps -> 2

  int len = 0xa8;
  if (now_ns - last_sent_packet_nanos >= 9000000)
  {
    last_sent_packet_nanos = now_ns;
    P2PPriority priority = P2PPriority::Level::kHigh;
    current_packet_view = p2p_stream.output().NewPacket(priority);
    if (current_packet_view.ok()) {
      for (int i = 0; i < len; ++i) {
        reinterpret_cast<uint8_t *>(current_packet_view->content())[i] = 0;
      }
      *reinterpret_cast<uint8_t *>(current_packet_view->content()) = sent_packets[priority];
      current_packet_view->length() = len; //sizeof(uint8_t);
      ASSERT(p2p_stream.output().Commit(priority, true));       

      ++sent_packets[priority];
      // ++len;
      // if (len == 0xa9) { len = 1; }
    }
  }

  len = 0xa8;
  P2PPriority priority = P2PPriority::Level::kLow;
  current_packet_view = p2p_stream.output().NewPacket(priority);
  if (current_packet_view.ok()) {
    for (int i = 0; i < len; ++i) {
      reinterpret_cast<uint8_t *>(current_packet_view->content())[i] = 0;
    }
    *reinterpret_cast<uint8_t *>(current_packet_view->content()) = sent_packets[priority];
    current_packet_view->length() = len; //sizeof(uint8_t);
    ASSERT(p2p_stream.output().Commit(priority, false)); 
    ++sent_packets[priority];
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
    Serial.print(", tx_delay(ns):");
    for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
      uint64_t delay = p2p_stream.output().stats().average_packet_delay_per_byte_ns(i);
      if (delay != -1ULL) {
        char str[24];
        Uint64ToString(delay, str);
        Serial.printf("%s ", str);
      } else {
        Serial.printf("? ");
      }
    }
    Serial.print(", rx_delay(ns):");
    for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
      uint64_t delay = p2p_stream.input().stats().average_packet_delay_per_byte_ns(i);
      if (delay != -1ULL) {
        char str[24];
        Uint64ToString(delay, str);
        Serial.printf("%s ", str);
      } else {
        Serial.printf("? ");
      }
    }
    Serial.println("");
    for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
      last_sent_packets[i] = sent_packets[i];
      last_received_packets[i] = received_packets[i];
    }
    last_msg_time_ns = now_ns;
  }

  if (p2p_stream.input().OldestPacket().ok()) {
    // Serial.write(p2p_input_stream.OldestPacket()->content(), p2p_input_stream.OldestPacket()->length());
    // Serial.print("p: ");
    // for (int i = 0; i < p2p_input_stream.OldestPacket()->length(); ++i) {
    //   Serial.printf("%x ", p2p_input_stream.OldestPacket()->content()[i]);
    // }
    // Serial.println();
    
    P2PPriority priority = p2p_stream.input().OldestPacket()->priority();
    ++received_packets[priority];
    if (last_received_packet_value[priority] >= 0) {
      int diff = p2p_stream.input().OldestPacket()->content()[0] - last_received_packet_value[priority];
      if (diff > 0) lost_packets[priority] += diff - 1;
      else lost_packets[priority] += 256 + diff - 1;
    }
    last_received_packet_value[priority] = p2p_stream.input().OldestPacket()->content()[0];
    p2p_stream.input().Consume(priority);
  }

  p2p_stream.input().Run();
  p2p_stream.output().Run();

  return;
  */
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


  uint64_t now_ns = GetTimerNanoseconds();
  float time_factor = kWaitSeconds / 3.0f;
  float elapsed_time_head_s = time_factor * ((now_ns - last_head_ns) / 1e9f);
  switch(head_state) {
    case LOOKING_FORWARD:
      SetHeadPitchDegrees(0);
      SetHeadRollDegrees(0);
      if (elapsed_time_head_s > 1 && elapsed_time_head_s < 1.6) {
        head_state = LOOKING_DOWN;
      }
      if (elapsed_time_head_s > 2.3) {
        head_state = LOOKING_DOWN;
      }
      break;
    case LOOKING_DOWN:
      SetHeadPitchDegrees(30);
      if (elapsed_time_head_s > 1.6 && elapsed_time_head_s < 2.3) {
        head_state = LOOKING_FORWARD;
      }
      if (elapsed_time_head_s >= 3) {
        head_state = LOOKING_UP;
      }
      break;
    case LOOKING_UP: {
      float deg = -35.0f / kWaitSeconds * (elapsed_time_head_s - 3);
      if (deg < -25) deg = -25;
      SetHeadPitchDegrees(deg);
      if (elapsed_time_head_s > 7) {
        head_state = TILTING_HEAD;
      }
      break;
    }
    case TILTING_HEAD:
      SetHeadRollDegrees(30);
      if (elapsed_time_head_s > 8) {
        head_state = HEAD_STOP;
      }
      break;
    case HEAD_STOP:
      SetHeadRollDegrees(0);
      break;
  }

  switch (state) {
    case INIT:
      SetLeftMotorDutyCycle(kInnerDutyCycle);
      SetRightMotorDutyCycle(1.0);
      // Disable the IRQ while writing to the event buffer to avoid a race.
      NO_TIMER_IRQ {
        event_buffer.Write(Event{ .type = kInnerDutyCycle >= 0 ? kLeftWheelForwardCommand : kLeftWheelBackwardCommand, .ticks = GetTimerTicks() });
        event_buffer.Write(Event{ .type = kRightWheelForwardCommand, .ticks = GetTimerTicks() });
      }
      last_timestamp_ns = GetTimerNanoseconds();
      state = CIRCLING_LEFT;
      break;
    case CIRCLING_LEFT:
      if (now_ns - last_timestamp_ns > (kWaitSeconds * 1000000000ULL)) {
        SetLeftMotorDutyCycle(1.0);
        SetRightMotorDutyCycle(kInnerDutyCycle);
        // Disable the IRQ while writing to the event buffer to avoid a race.
        NO_TIMER_IRQ {
          event_buffer.Write(Event{ .type = kLeftWheelForwardCommand, .ticks = GetTimerTicks() });
          event_buffer.Write(Event{ .type = kInnerDutyCycle >= 0 ? kRightWheelForwardCommand : kRightWheelBackwardCommand, .ticks = GetTimerTicks() });
        }
        last_timestamp_ns = now_ns;
        state = CIRCLING_RIGHT;
      }
      break;
    case CIRCLING_RIGHT:
      if (now_ns - last_timestamp_ns > (kWaitSeconds * 0.8f * 1000000000ULL)) {
        SetLeftMotorDutyCycle(1);
        SetRightMotorDutyCycle(1);
        last_timestamp_ns = now_ns;
        state = SLOW_DOWN;
      }
      break;
    case SLOW_DOWN: {
      float dc = 1 - (now_ns - last_timestamp_ns)/1e9f;
      if (dc < 0) dc = 0;
      SetLeftMotorDutyCycle(dc);
      SetRightMotorDutyCycle(dc);
      if (now_ns - last_timestamp_ns > (kWaitSeconds * 0.8f * 1000000000ULL)) {
        SetLeftMotorDutyCycle(0);
        SetRightMotorDutyCycle(0);
        last_timestamp_ns = now_ns;
        state = WAIT_FOR_INPUT;
      }
      break;
    }
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
      // Disable the IRQ while reading from the event buffer to avoid a race.
      Event event;
      NO_TIMER_IRQ {
        event = event_buffer.Read();
      }
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

  //  SetLeftMotorDutyCycle(1.0f);
  //  SetRightMotorDutyCycle(1.0f);
}

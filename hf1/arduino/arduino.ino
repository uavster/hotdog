// This program assumes the CPU clock is set to 96 MHz.

#include "utils.h"
#include "p2p_byte_stream_arduino.h"
#include "p2p_packet_stream.h"
#include "motors.h"
#include "servos.h"
#include "timer.h"
#include "encoders.h"
#include "timer_arduino.h"
#include "guid_factory.h"
#include "logger.h"
#include "time_sync_server.h"
#include "robot_state_estimator.h"

#define kP2PInputCapacity 4
#define kP2POutputCapacity 1
#define kP2PLocalEndianness kLittleEndian

Logger logger;

// #include <SPI.h>

// #define kAutomaticTransitions

P2PByteStreamArduino byte_stream(&Serial1);
TimerArduino timer;
GUIDFactory guid_factory;
// P2PPacketInputStream<8, kLittleEndian> p2p_input_stream(&byte_stream, &timer);
// P2PPacketOutputStream<1, kLittleEndian> p2p_output_stream(&byte_stream, &timer);
P2PPacketStream<kP2PInputCapacity, kP2POutputCapacity, kP2PLocalEndianness> p2p_stream(&byte_stream, &timer, guid_factory);
TimeSyncServer<kP2PInputCapacity, kP2POutputCapacity, kP2PLocalEndianness> time_sync_server(&p2p_stream, &timer);

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

  *logger.base_logger() = SetLogger(&logger);

  InitTimer();

  // Serial starts working after some time. Wait, so we don't miss any log.
  while(GetTimerNanoseconds() < 3000000000ULL) {}

  Serial.println("Initialized debugging serial port and timing modules.");

  Serial.println("Initializing robot state estimator...");
  InitRobotStateEstimator();

  Serial.println("Initializing inter-board communications...");
  // Serial1.addMemoryForRead(rx_buffer, 256);
  Serial1.begin(1000000, SERIAL_8N1);

  Serial.println("Initializing motors...");
  InitMotors();

  Serial.println("Initializing servos...");
  InitServos();

  for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
    last_received_packet_value[i] = -1;
    received_packets[i] = 0;
    sent_packets[i] = 0;
    last_received_packets[i] = 0;
    last_sent_packets[i] = 0;
    lost_packets[i] = 0;
  }

  Serial.println("Initializing time server...");
  time_sync_server.Init();

  last_msg_time_ns = GetTimerNanoseconds();

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
  
  Serial.println("Ready.");
}

//char format_buffer[32];

// enum { INIT,
//        LOOK_LEFT,
//        LOOK_RIGHT,
//        LOOK_CENTER,
//        MOVE_FORWARD,
//        DONE } state = INIT;
uint64_t last_timestamp_ns = 0;

#define kWaitSeconds 3
#define kWaitSecondsDrive 0.25
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

typedef enum {
  IDLE = 0,
  NODDING = 1,
  NEGATING = 2,
  TURNING_LEFT = 3,
  TURNING_RIGHT = 4,
  LOOKING_UP = 5,
  LOOKING_DOWN = 6,
  MOVING_FORWARD = 7,
  MOVING_BACKWARD = 8,
  // LOOKING_UP,
  // TILTING_HEAD,
  // HEAD_STOP
} HeadState;

HeadState head_state = IDLE;

uint64_t next_global_time_event_s = -1ULL;

typedef struct {
  uint64_t timestamp_ns;
  float pitch;
  float roll;
} HeadPose;

HeadPose next_idle_head_pose{0, 0, 0};
uint64_t nodding_start_time_ns = -1ULL; 
uint64_t nodding_stop_time_ns = -1ULL;

uint64_t negating_start_time_ns = -1ULL;
uint64_t negating_stop_time_ns = -1ULL;

uint64_t next_wheel_command_time_ns = 0;
float next_wheel_command_left_dc = 0;
float next_wheel_command_right_dc = 0;

void loop() {
  RunRobotStateEstimator();
  Serial.printf("cx:%f cy:%f a:%f vx:%f vy:%f\n", GetRobotState().Center().x, GetRobotState().Center().y, GetRobotState().Angle(), GetRobotState().CenterVelocity().x, GetRobotState().CenterVelocity().y);

  p2p_stream.input().Run();
  p2p_stream.output().Run();
  time_sync_server.Run();

  uint64_t now_ns = GetTimerNanoseconds();

  const uint64_t nodding_period_ns = 100000000ULL;
  const uint64_t negating_period_ns = 200000000ULL;

  if (p2p_stream.input().OldestPacket().ok()) {
    P2PPriority priority = p2p_stream.input().OldestPacket()->priority();
    if (p2p_stream.input().OldestPacket()->content()[0] == 3) {
      uint8_t command = p2p_stream.input().OldestPacket()->content()[1];
      head_state = static_cast<HeadState>(command);
      switch(head_state) {
        case NODDING:
          next_idle_head_pose.pitch = -20 + 10;
          next_idle_head_pose.roll = 0;
          next_idle_head_pose.timestamp_ns = now_ns + nodding_period_ns;
          nodding_stop_time_ns = now_ns + 1 * 1000000000ULL;
          break;
        case NEGATING:
          next_wheel_command_time_ns = now_ns + negating_period_ns;
          next_wheel_command_left_dc = 0.5;
          next_wheel_command_right_dc = -0.5;
          negating_stop_time_ns = now_ns + 2 * 1000000000ULL;
          break;
        case TURNING_RIGHT:
          next_wheel_command_left_dc = -0.5;
          next_wheel_command_right_dc = 0.5;
          negating_stop_time_ns = now_ns + 1 * 1000000000ULL;
          break;
        case TURNING_LEFT:
          next_wheel_command_left_dc = 0.5;
          next_wheel_command_right_dc = -0.5 - 0.08;
          negating_stop_time_ns = now_ns + 1 * 1000000000ULL;
          break;
        case LOOKING_UP:
          SetHeadPitchDegrees(-40);
          break;
        case LOOKING_DOWN:
          SetHeadPitchDegrees(40);
          break;
        case MOVING_FORWARD:
          next_wheel_command_left_dc = 0.5;
          next_wheel_command_right_dc = 0.5;
          negating_stop_time_ns = now_ns + 1 * 1000000000ULL;
          break;
        case MOVING_BACKWARD:
          next_wheel_command_left_dc = -0.5;
          next_wheel_command_right_dc = -0.5;
          negating_stop_time_ns = now_ns + 1 * 1000000000ULL;
          break;
        default:
          head_state = IDLE;
          break;
      }
      p2p_stream.input().Consume(priority);
    }
  }


  // uint64_t now_ns = GetTimerNanoseconds();
  // if (now_ns - last_ns >= 1000000000) {
  //   char tmp[64];
  //   Uint64ToString(now_ns, tmp);
  //   Serial.println(tmp);
  //   last_ns = now_ns;
  // }

  // return;

/*  
  if (next_global_time_event_s == -1ULL) {
    next_global_time_event_s = (timer.GetGlobalNanoseconds() + 1000000000ULL) / 1000000000ULL;
  }
  uint64_t cur_global_time_ns = timer.GetGlobalNanoseconds();
	if (cur_global_time_ns / 1000000000ULL >= next_global_time_event_s) {
    char tmp[32];
    Uint64ToString(cur_global_time_ns, tmp);
    Serial.println(tmp);
    next_global_time_event_s = (cur_global_time_ns + 1000000000ULL) / 1000000000ULL;
  }

  p2p_stream.input().Run();
  p2p_stream.output().Run();
  time_sync_server.Run();

  return;
*/

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


#ifdef kAutomaticTransitions
  if (nodding_start_time_ns == -1ULL) {
    nodding_start_time_ns = now_ns + 15 * 1000000000ULL;      
    nodding_stop_time_ns = now_ns + 16 * 1000000000ULL;
    negating_start_time_ns = now_ns + 12 * 1000000000ULL;
    negating_stop_time_ns = now_ns + 13 * 1000000000ULL;
  }
#endif  
  // float time_factor = kWaitSeconds / 3.0f;
  // float elapsed_time_head_s = time_factor * ((now_ns - last_head_ns) / 1e9f);
  switch(head_state) {
    case IDLE:
      SetHeadPitchDegrees(next_idle_head_pose.pitch);
      SetHeadRollDegrees(next_idle_head_pose.roll);
      if (now_ns > next_idle_head_pose.timestamp_ns) {
        next_idle_head_pose.timestamp_ns = now_ns + 1000000ULL * (1000 + random(0, 200));
        next_idle_head_pose.pitch = -20 + (random(0, 4 * 1000) - 1 * 1000) / 1000.0f;
        next_idle_head_pose.roll = (random(0, 4 * 1000) - 1 * 1000) / 1000.0f;
      }
#ifdef kAutomaticTransitions      
      if (now_ns > nodding_start_time_ns) {
        next_idle_head_pose.pitch = -20 + 10;
        next_idle_head_pose.roll = 0;
        next_idle_head_pose.timestamp_ns = now_ns + nodding_period_ns;
        head_state = NODDING;
      }
      if (now_ns > negating_start_time_ns) {
        next_wheel_command_time_ns = now_ns + negating_period_ns;
        next_wheel_command_left_dc = 0.5;
        next_wheel_command_right_dc = -0.5 + 0.08;
        head_state = NEGATING;
      }
#endif    
      break;

    case NODDING:
      SetHeadPitchDegrees(next_idle_head_pose.pitch);
      SetHeadRollDegrees(next_idle_head_pose.roll);
      if (now_ns > next_idle_head_pose.timestamp_ns) {
        next_idle_head_pose.timestamp_ns = now_ns + nodding_period_ns;
        if (next_idle_head_pose.pitch != -20 + 10) {
          next_idle_head_pose.pitch = -20 + 10;
        } else {
          next_idle_head_pose.pitch = -20;
        }
      }
      if (now_ns > nodding_stop_time_ns) {
#ifdef kAutomaticTransitions 
        next_idle_head_pose.timestamp_ns = 0;
        nodding_start_time_ns = negating_start_time_ns + 3 * 1000000000ULL;      
        nodding_stop_time_ns = negating_start_time_ns + 4 * 1000000000ULL;
#endif      
        head_state = IDLE;
      }
      break;

    case NEGATING:
      SetLeftMotorDutyCycle(next_wheel_command_left_dc);
      SetRightMotorDutyCycle(next_wheel_command_right_dc);
      if (now_ns > next_wheel_command_time_ns) {
#ifdef kAutomaticTransitions        
        next_wheel_command_time_ns = now_ns + negating_period_ns;
#endif        
        if (next_wheel_command_left_dc > 0) {
          next_wheel_command_left_dc = -0.5;
        } else {
          next_wheel_command_left_dc = 0.5;
        }
        if (next_wheel_command_right_dc > 0) {
          next_wheel_command_right_dc = -0.5 + 0.08;
        } else {
          next_wheel_command_right_dc = 0.5 + 0.08;
        }
      }
      if (now_ns > negating_stop_time_ns) {
        SetLeftMotorDutyCycle(0);
        SetRightMotorDutyCycle(0);
#ifdef kAutomaticTransitions        
        next_wheel_command_time_ns = 0;
        negating_start_time_ns = nodding_start_time_ns + 12 * 1000000000ULL;      
        negating_stop_time_ns = nodding_start_time_ns + 13 * 1000000000ULL;
#endif        
        head_state = IDLE;
      }
      break;

    case TURNING_RIGHT:
    case TURNING_LEFT:
    case MOVING_FORWARD:
    case MOVING_BACKWARD:
      SetLeftMotorDutyCycle(next_wheel_command_left_dc);
      SetRightMotorDutyCycle(next_wheel_command_right_dc);
      if (now_ns > negating_stop_time_ns) {
        SetLeftMotorDutyCycle(0);
        SetRightMotorDutyCycle(0);
        head_state = IDLE;
      }
      break;      

    case LOOKING_UP:
    case LOOKING_DOWN:
      break;
  }

  // switch (state) {
  //   case INIT:
  //     SetLeftMotorDutyCycle(0);
  //     SetRightMotorDutyCycle(0);
  //     if (now_ns - last_timestamp_ns > (kWaitSecondsDrive * 1000000000ULL)) {
  //       last_timestamp_ns = now_ns;
  //       state = LOOK_LEFT;
  //     }
  //     break;
  //   case LOOK_LEFT:
  //     if (now_ns - last_timestamp_ns < (kWaitSecondsDrive * 1000000000ULL)) {
  //       SetLeftMotorDutyCycle(-0.5);
  //       SetRightMotorDutyCycle(0.5);
  //     }
  //     if (now_ns - last_timestamp_ns >= (kWaitSecondsDrive * 1000000000ULL)) {
  //       SetLeftMotorDutyCycle(0);
  //       SetRightMotorDutyCycle(0);
  //     }
  //     if (now_ns - last_timestamp_ns > (4 * kWaitSecondsDrive * 1000000000ULL)) {
  //       last_timestamp_ns = now_ns;
  //       state = LOOK_RIGHT;
  //     }
  //     break;
  //   case LOOK_RIGHT:
  //     if (now_ns - last_timestamp_ns < (kWaitSecondsDrive * 1000000000ULL)) {
  //       SetLeftMotorDutyCycle(0.5);
  //       SetRightMotorDutyCycle(-0.5);
  //     }
  //     if (now_ns - last_timestamp_ns >= (2 * kWaitSecondsDrive * 1000000000ULL)) {
  //       SetLeftMotorDutyCycle(0);
  //       SetRightMotorDutyCycle(0);
  //     }
  //     if (now_ns - last_timestamp_ns > (4 * kWaitSecondsDrive * 1000000000ULL)) {
  //       last_timestamp_ns = now_ns;
  //       state = LOOK_CENTER;
  //     }
  //     break;
  //   case LOOK_CENTER:
  //     SetLeftMotorDutyCycle(-0.5);
  //     SetRightMotorDutyCycle(0.5);
  //     if (now_ns - last_timestamp_ns > (kWaitSecondsDrive * 1000000000ULL)) {
  //       last_timestamp_ns = now_ns;
  //       state = MOVE_FORWARD;
  //     }
  //     break;
  //   case MOVE_FORWARD: {
  //     float dc = 0.5 * (now_ns - last_timestamp_ns) / (kWaitSecondsDrive * 10 * 1000000000ULL);
  //     SetLeftMotorDutyCycle(dc);
  //     SetRightMotorDutyCycle(dc);
  //     if (now_ns - last_timestamp_ns > (kWaitSecondsDrive * 20 * 1000000000ULL)) {
  //       last_timestamp_ns = now_ns;
  //       state = INIT;
  //     }
  //     break;
  //   }
  //   case DONE:
  //     break;
  // }

  //  SetLeftMotorDutyCycle(1.0f);
  //  SetRightMotorDutyCycle(1.0f);
}

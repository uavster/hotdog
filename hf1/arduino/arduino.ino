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
#include "robot_state_estimator.h"
#include "wheel_controller.h"
#include "base_controller.h"
#include "p2p_packet_stream_arduino.h"
#include "p2p_action_server.h"
#include "sync_time_action_handler.h"
#include "set_head_pose_action_handler.h"
#include "set_base_velocity_action_handler.h"
#include "monitor_base_state_action_handler.h"

Logger logger;

// #include <SPI.h>

// #define kAutomaticTransitions

// Maximum time during which communication can be processed without
// yielding time to other tasks. 
#define kMaxRxTxLoopBlockingDurationNs 100'000'000

P2PByteStreamArduino byte_stream(&Serial1);
TimerArduino timer;
GUIDFactory guid_factory;
P2PPacketStreamArduino p2p_stream(&byte_stream, &timer, guid_factory);

TimerNanosType last_msg_time_ns = 0;

int last_received_packet_value[P2PPriority::kNumLevels];
int received_packets[P2PPriority::kNumLevels];
int sent_packets[P2PPriority::kNumLevels];
int last_received_packets[P2PPriority::kNumLevels];
int last_sent_packets[P2PPriority::kNumLevels];
int lost_packets[P2PPriority::kNumLevels];

WheelSpeedController left_wheel(&GetLeftWheelTickCount, &SetLeftMotorDutyCycle);
WheelSpeedController right_wheel(&GetRightWheelTickCount, &SetRightMotorDutyCycle);
BaseSpeedController base_speed_controller(&left_wheel, &right_wheel);
BaseStateController base_state_controller(&base_speed_controller);
BaseTrajectoryController base_trajectory_controller(&base_speed_controller);

P2PActionServer p2p_action_server(&p2p_stream);
SetHeadPoseActionHandler set_head_pose_action_handler(&p2p_stream);
SetBaseVelocityActionHandler set_base_velocity_action_handler(&p2p_stream, &base_speed_controller);
SyncTimeActionHandler sync_time_action_handler(&p2p_stream, &timer);
MonitorBaseStateActionHandler monitor_base_state_action_handler(&p2p_stream, &timer);

// const BaseWaypoint waypoints[] = { 
//   BaseWaypoint(0, Point(0, 0), 0), 
//   BaseWaypoint(2.7, Point(1-0.3/3, 0), M_PI/2), 
//   BaseWaypoint(3, Point(1, 0), M_PI/2), 
//   BaseWaypoint(3.3, Point(1, -0.3/3), M_PI/2), 
//   BaseWaypoint(5.7, Point(1, -1+0.3/3), M_PI/2), 
//   BaseWaypoint(6, Point(1, -1), M_PI), 
//   BaseWaypoint(6.3, Point(1-0.3/3, -1), M_PI), 
//   BaseWaypoint(8.7, Point(0+0.3/3, -1), M_PI), 
//   BaseWaypoint(9, Point(0, -1), -M_PI/2), 
//   BaseWaypoint(9.3, Point(0, -1+0.3/3), -M_PI/2), 
//   BaseWaypoint(11.7, Point(0, 0-0.3/3), -M_PI/2), 
//   BaseWaypoint(12, Point(0, 0), 0) };

BaseWaypoint waypoints[80];

void setup() {
  // Open serial port before anything else, as it enables showing logs and asserts in the console.
  Serial.begin(115200);

  *logger.base_logger() = SetLogger(&logger);

  InitTimer();

  // Serial starts working after some time. Wait, so we don't miss any log.
  while(GetTimerNanoseconds() < 3000000000ULL) {}

  Serial.println("Initialized debugging serial port and timing modules.");

  Serial.println("Initializing encoders...");
  InitEncoders();
  
  Serial.println("Initializing wheel speed estimator...");
  InitWheelSpeedControl();

  Serial.println("Initializing robot state estimator...");
  InitRobotStateEstimator();

  Serial.println("Initializing inter-board communications...");
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

  Serial.println("Register actions in action server...");
  p2p_action_server.Register(&sync_time_action_handler);
  p2p_action_server.Register(&set_head_pose_action_handler);
  p2p_action_server.Register(&set_base_velocity_action_handler);
  p2p_action_server.Register(&monitor_base_state_action_handler);

  last_msg_time_ns = GetTimerNanoseconds();
  
  Serial.println("Ready.");

  // base_state_controller.SetTargetState(Point(0.5, 0.5), M_PI / 4, 0.3, 0);  
  // base_speed_controller.SetTargetSpeeds(0.1, 10 * M_PI);

  // const int num_waypoints = 40;
  // const int points_per_segment = num_waypoints / 4;
  // for (int i = 0; i < points_per_segment; ++i) {
  //   waypoints[i] = BaseWaypoint(i * 0.3, Point(i * 0.1, 0));
  //   waypoints[i+points_per_segment] = BaseWaypoint((i+points_per_segment) * 0.3, Point(1, -i * 0.1));
  //   waypoints[i+2*points_per_segment] = BaseWaypoint((i+2*points_per_segment) * 0.3, Point(1 - i * 0.1, -1));
  //   waypoints[i+3*points_per_segment] = BaseWaypoint((i+3*points_per_segment) * 0.3, Point(0, -1+0.1*i));
  // }

  const int num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);
  const float total_trajectory_seconds = 20.0;
  for (int i = 0; i < num_waypoints; ++i) {
    const float t = i * total_trajectory_seconds / num_waypoints;
    const float w = 2 * M_PI / total_trajectory_seconds;
    const float x = sin(w * t);
    const float y = -1 + cos(w * t);
    waypoints[i] = BaseWaypoint(t, BaseTargetState({ BaseStateVars(Point(x, y), 0) }));
  }

  base_trajectory_controller.trajectory(BaseTrajectoryView(num_waypoints, waypoints).EnableLooping(/*after_seconds=*/1.0));
  base_trajectory_controller.StartTrajectory();
  // Serial.printf("target_linear_speed:%f target_angular_speed:%f radius:%f\n", base_speed_controller.target_linear_speed(), base_speed_controller.target_angular_speed(), base_speed_controller.curve_radius());
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

class SpeedControllerTest : public PeriodicRunnable {
public:
  SpeedControllerTest(BaseSpeedController *base_controller) : PeriodicRunnable(0.33), base_controller_(base_controller), linear_speed_(0) {}

  void RunFirstTime(TimerNanosType now_nanos) {
    const float linear_speed = -0.3;
    // const float radius = 1.0;
    // base_controller_->SetTargetSpeeds(linear_speed, 0); // linear_speed / radius);
  }
  void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override {
    const float linear_speed = -0.3;
    const float radius = 1.0;
    base_controller_->SetTargetSpeeds(linear_speed, 0);
    // base_controller_->SetTargetSpeeds(linear_speed * sin(2 * M_PI * 0.1 * SecondsFromNanos(now_nanos)), 0); //linear_speed / radius);
    // linear_speed_ += 0.1 * SecondsFromNanos(nanos_since_last_call);
    // const float target_angular = M_PI;
    // base_controller_->SetTargetSpeeds(linear_speed_, target_angular);
    // // Serial.printf("linear:%f->%f angular:%f->%f radius:%f\n", linear_speed_, base_controller_->target_linear_speed(), target_angular, base_controller_->target_angular_speed(), base_controller_->curve_radius());
  }

private:
  BaseSpeedController *base_controller_;
  float linear_speed_;
};

SpeedControllerTest speed_controller_test(&base_speed_controller);

void loop() {
  RunRobotStateEstimator();
  // Serial.printf("%d|cx:%f cy:%f a:%f vx:%f vy:%f\n", base_state_controller.IsAtTargetState() ? 1:0, GetBaseState().center().x, GetBaseState().center().y, GetBaseState().yaw(), GetBaseState().center_velocity().x, GetBaseState().center_velocity().y);

  // if (!base_state_controller.IsAtTargetState()) {
  //   base_state_controller.Run();
  // } else {
  //   base_speed_controller.SetTargetSpeeds(0, 0);
  // }

  base_trajectory_controller.Run();
  NotifyLeftMotorDirection(GetTimerTicks(), !base_trajectory_controller.base_speed_controller().left_wheel_speed_controller().is_turning_forward());
  NotifyRightMotorDirection(GetTimerTicks(), !base_trajectory_controller.base_speed_controller().right_wheel_speed_controller().is_turning_forward());
  
  // speed_controller_test.Run();
  // base_speed_controller.Run();

  bool process_comms = true;
  const auto process_comms_start_time_ns = timer.GetLocalNanoseconds();
  while(process_comms) {
    // Keep processing communications while there are bytes read
    // or ready to send.
    process_comms = p2p_stream.input().Run() > 0;
    p2p_stream.output().Run();
    p2p_action_server.Run();
    process_comms = process_comms || p2p_stream.output().NumCommittedPackets() > 0;
    // If processing for too long, yield time to other tasks.
    process_comms = process_comms && (timer.GetLocalNanoseconds() - process_comms_start_time_ns > kMaxRxTxLoopBlockingDurationNs);
  }

  // uint64_t now_ns = GetTimerNanoseconds();

  // const uint64_t nodding_period_ns = 100000000ULL;
  // const uint64_t negating_period_ns = 200000000ULL;

  // if (p2p_stream.input().OldestPacket().ok()) {
  //   // Serial.printf("Got packet\n");
  //   P2PPriority priority = p2p_stream.input().OldestPacket()->priority();
  //   if (p2p_stream.input().OldestPacket()->content()[0] == 3) {
  //     uint8_t command = p2p_stream.input().OldestPacket()->content()[1];
  //     head_state = static_cast<HeadState>(command);
  //     switch(head_state) {
  //       case NODDING:
  //         next_idle_head_pose.pitch = -20 + 10;
  //         next_idle_head_pose.roll = 0;
  //         next_idle_head_pose.timestamp_ns = now_ns + nodding_period_ns;
  //         nodding_stop_time_ns = now_ns + 1 * 1000000000ULL;
  //         break;
  //       case NEGATING:
  //         next_wheel_command_time_ns = now_ns + negating_period_ns;
  //         next_wheel_command_left_dc = 0.5;
  //         next_wheel_command_right_dc = -0.5;
  //         negating_stop_time_ns = now_ns + 2 * 1000000000ULL;
  //         break;
  //       case TURNING_RIGHT:
  //         next_wheel_command_left_dc = -0.5;
  //         next_wheel_command_right_dc = 0.5;
  //         negating_stop_time_ns = now_ns + 1 * 1000000000ULL;
  //         break;
  //       case TURNING_LEFT:
  //         next_wheel_command_left_dc = 0.5;
  //         next_wheel_command_right_dc = -0.5 - 0.08;
  //         negating_stop_time_ns = now_ns + 1 * 1000000000ULL;
  //         break;
  //       case LOOKING_UP:
  //         SetHeadPitchDegrees(-40);
  //         break;
  //       case LOOKING_DOWN:
  //         SetHeadPitchDegrees(40);
  //         break;
  //       case MOVING_FORWARD:
  //         next_wheel_command_left_dc = 0.5;
  //         next_wheel_command_right_dc = 0.5;
  //         negating_stop_time_ns = now_ns + 1 * 1000000000ULL;
  //         break;
  //       case MOVING_BACKWARD:
  //         next_wheel_command_left_dc = -0.5;
  //         next_wheel_command_right_dc = -0.5;
  //         negating_stop_time_ns = now_ns + 1 * 1000000000ULL;
  //         break;
  //       default:
  //         head_state = IDLE;
  //         break;
  //     }
  //     p2p_stream.input().Consume(priority);
  //   }
  // }


  // uint64_t now_ns = GetTimerNanoseconds();
  // if (now_ns - last_ns >= 1000000000) {
  //   char tmp[64];
  //   Uint64ToString(now_ns, tmp);
  //   Serial.println(tmp);
  //   last_ns = now_ns;
  // }

  // return;

// #ifdef kAutomaticTransitions
//   if (nodding_start_time_ns == -1ULL) {
//     nodding_start_time_ns = now_ns + 15 * 1000000000ULL;      
//     nodding_stop_time_ns = now_ns + 16 * 1000000000ULL;
//     negating_start_time_ns = now_ns + 12 * 1000000000ULL;
//     negating_stop_time_ns = now_ns + 13 * 1000000000ULL;
//   }
// #endif  
//   // float time_factor = kWaitSeconds / 3.0f;
//   // float elapsed_time_head_s = time_factor * ((now_ns - last_head_ns) / 1e9f);
//   switch(head_state) {
//     case IDLE:
//       SetHeadPitchDegrees(next_idle_head_pose.pitch);
//       SetHeadRollDegrees(next_idle_head_pose.roll);
//       if (now_ns > next_idle_head_pose.timestamp_ns) {
//         next_idle_head_pose.timestamp_ns = now_ns + 1000000ULL * (1000 + random(0, 200));
//         next_idle_head_pose.pitch = -20 + (random(0, 4 * 1000) - 1 * 1000) / 1000.0f;
//         next_idle_head_pose.roll = (random(0, 4 * 1000) - 1 * 1000) / 1000.0f;
//       }
// #ifdef kAutomaticTransitions      
//       if (now_ns > nodding_start_time_ns) {
//         next_idle_head_pose.pitch = -20 + 10;
//         next_idle_head_pose.roll = 0;
//         next_idle_head_pose.timestamp_ns = now_ns + nodding_period_ns;
//         head_state = NODDING;
//       }
//       if (now_ns > negating_start_time_ns) {
//         next_wheel_command_time_ns = now_ns + negating_period_ns;
//         next_wheel_command_left_dc = 0.5;
//         next_wheel_command_right_dc = -0.5 + 0.08;
//         head_state = NEGATING;
//       }
// #endif    
//       break;

//     case NODDING:
//       SetHeadPitchDegrees(next_idle_head_pose.pitch);
//       SetHeadRollDegrees(next_idle_head_pose.roll);
//       if (now_ns > next_idle_head_pose.timestamp_ns) {
//         next_idle_head_pose.timestamp_ns = now_ns + nodding_period_ns;
//         if (next_idle_head_pose.pitch != -20 + 10) {
//           next_idle_head_pose.pitch = -20 + 10;
//         } else {
//           next_idle_head_pose.pitch = -20;
//         }
//       }
//       if (now_ns > nodding_stop_time_ns) {
// #ifdef kAutomaticTransitions 
//         next_idle_head_pose.timestamp_ns = 0;
//         nodding_start_time_ns = negating_start_time_ns + 3 * 1000000000ULL;      
//         nodding_stop_time_ns = negating_start_time_ns + 4 * 1000000000ULL;
// #endif      
//         head_state = IDLE;
//       }
//       break;

//     case NEGATING:
//       SetLeftMotorDutyCycle(next_wheel_command_left_dc);
//       SetRightMotorDutyCycle(next_wheel_command_right_dc);
//       if (now_ns > next_wheel_command_time_ns) {
// #ifdef kAutomaticTransitions        
//         next_wheel_command_time_ns = now_ns + negating_period_ns;
// #endif        
//         if (next_wheel_command_left_dc > 0) {
//           next_wheel_command_left_dc = -0.5;
//         } else {
//           next_wheel_command_left_dc = 0.5;
//         }
//         if (next_wheel_command_right_dc > 0) {
//           next_wheel_command_right_dc = -0.5 + 0.08;
//         } else {
//           next_wheel_command_right_dc = 0.5 + 0.08;
//         }
//       }
//       if (now_ns > negating_stop_time_ns) {
//         SetLeftMotorDutyCycle(0);
//         SetRightMotorDutyCycle(0);
// #ifdef kAutomaticTransitions        
//         next_wheel_command_time_ns = 0;
//         negating_start_time_ns = nodding_start_time_ns + 12 * 1000000000ULL;      
//         negating_stop_time_ns = nodding_start_time_ns + 13 * 1000000000ULL;
// #endif        
//         head_state = IDLE;
//       }
//       break;

//     case TURNING_RIGHT:
//     case TURNING_LEFT:
//     case MOVING_FORWARD:
//     case MOVING_BACKWARD:
//       SetLeftMotorDutyCycle(next_wheel_command_left_dc);
//       SetRightMotorDutyCycle(next_wheel_command_right_dc);
//       if (now_ns > negating_stop_time_ns) {
//         SetLeftMotorDutyCycle(0);
//         SetRightMotorDutyCycle(0);
//         head_state = IDLE;
//       }
//       break;      

//     case LOOKING_UP:
//     case LOOKING_DOWN:
//       break;
//   }

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

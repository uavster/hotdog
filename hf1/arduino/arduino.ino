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

// Maximum time during which communication can be processed without
// yielding time to other tasks. 
#define kMaxRxTxLoopBlockingDurationNs 100'000'000

Logger logger;

P2PByteStreamArduino byte_stream(&Serial1);
TimerArduino timer;
GUIDFactory guid_factory;
P2PPacketStreamArduino p2p_stream(&byte_stream, &timer, guid_factory);

WheelSpeedController left_wheel(&GetLeftWheelTickCount, &SetLeftMotorDutyCycle);
WheelSpeedController right_wheel(&GetRightWheelTickCount, &SetRightMotorDutyCycle);
BaseSpeedController base_speed_controller(&left_wheel, &right_wheel);
BaseTrajectoryController base_trajectory_controller(&base_speed_controller);

P2PActionServer p2p_action_server(&p2p_stream);
SetHeadPoseActionHandler set_head_pose_action_handler(&p2p_stream);
SetBaseVelocityActionHandler set_base_velocity_action_handler(&p2p_stream, &base_speed_controller);
SyncTimeActionHandler sync_time_action_handler(&p2p_stream, &timer);
MonitorBaseStateActionHandler monitor_base_state_action_handler(&p2p_stream, &timer);

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

  Serial.println("Register actions in action server...");
  p2p_action_server.Register(&sync_time_action_handler);
  p2p_action_server.Register(&set_head_pose_action_handler);
  p2p_action_server.Register(&set_base_velocity_action_handler);
  p2p_action_server.Register(&monitor_base_state_action_handler);

  Serial.println("Ready.");

  // base_state_controller.SetTargetState(Point(0.5, 0.5), M_PI / 4, 0.3, 0);  
  // base_speed_controller.SetTargetSpeeds(0.1, 10 * M_PI);

  // const int num_waypoints = 40;
  // const int points_per_segment = num_waypoints / 4;
  // for (int i = 0; i < points_per_segment; ++i) {
  //   waypoints[i] = BaseWaypoint(i * 0.3, BaseTargetState({ BaseStateVars(Point(i * 0.1, 0), 0) }));
  //   waypoints[i+points_per_segment] = BaseWaypoint((i+points_per_segment) * 0.3, BaseTargetState({ BaseStateVars(Point(1, -i * 0.1), 0) }));
  //   waypoints[i+2*points_per_segment] = BaseWaypoint((i+2*points_per_segment) * 0.3, BaseTargetState({ BaseStateVars(Point(1 - i * 0.1, -1), 0) }));
  //   waypoints[i+3*points_per_segment] = BaseWaypoint((i+3*points_per_segment) * 0.3, BaseTargetState({ BaseStateVars(Point(0, -1+0.1*i), 0) }));
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

  base_trajectory_controller.trajectory(BaseTrajectoryView(num_waypoints, waypoints).EnableLooping(/*after_seconds=*/5.0));
  base_trajectory_controller.StartTrajectory();
}

void loop() {
  RunRobotStateEstimator();

  base_trajectory_controller.Run();
  NotifyLeftMotorDirection(GetTimerTicks(), !base_trajectory_controller.base_speed_controller().left_wheel_speed_controller().is_turning_forward());
  NotifyRightMotorDirection(GetTimerTicks(), !base_trajectory_controller.base_speed_controller().right_wheel_speed_controller().is_turning_forward());
  
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
}

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
#include "wheel_state_estimator.h"
#include "wheel_controller.h"
#include "robot_state_estimator.h"
#include "base_controller.h"
#include "head_controller.h"
#include "p2p_packet_stream_arduino.h"
#include "p2p_action_server.h"
#include "sync_time_action_handler.h"
#include "set_head_pose_action_handler.h"
#include "set_base_velocity_action_handler.h"
#include "monitor_base_state_action_handler.h"

// Maximum time during which communication can be processed without
// yielding time to other tasks.
// This should be the minium period of all control loops.
#define kMaxRxTxLoopBlockingDurationNs 5'000'000

Logger logger;

P2PByteStreamArduino byte_stream(&Serial1);
TimerArduino timer;
GUIDFactory guid_factory;
P2PPacketStreamArduino p2p_stream(&byte_stream, &timer, guid_factory);

WheelStateEstimator wheel_state_estimator("WheelStateEstimator");
WheelSpeedController left_wheel("LeftWheelSpeedController", &wheel_state_estimator.left_wheel_state_filter(), &SetLeftMotorDutyCycle);
WheelSpeedController right_wheel("RightWheelSpeedController", &wheel_state_estimator.right_wheel_state_filter(), &SetRightMotorDutyCycle);
BaseSpeedController base_speed_controller(&left_wheel, &right_wheel);
BaseTrajectoryController base_trajectory_controller("BaseTrajectoryController", &base_speed_controller);
HeadTrajectoryController head_trajectory_controller("HeadTrajectoryController");

P2PActionServer p2p_action_server(&p2p_stream);
SetHeadPoseActionHandler set_head_pose_action_handler(&p2p_stream);
SetBaseVelocityActionHandler set_base_velocity_action_handler(&p2p_stream, &base_speed_controller);
SyncTimeActionHandler sync_time_action_handler(&p2p_stream, &timer);
MonitorBaseStateActionHandler monitor_base_state_action_handler(&p2p_stream, &timer);

Trajectory<BaseTargetState, 40> base_carrier;
Trajectory<BaseTargetState, 10> base_modulator;
Trajectory<EnvelopeTargetState, 10> base_envelope;

Trajectory<HeadTargetState, 10> head_carrier;
Trajectory<HeadTargetState, 10> head_modulator;
Trajectory<EnvelopeTargetState, 10> head_envelope;

void setup() {
  // Open serial port before anything else, as it enables showing logs and asserts in the console.
  Serial.begin(115200);

  *logger.base_logger() = SetLogger(&logger);

  InitTimer();

  // Serial starts working after some time. Wait, so we don't miss any log.
  while(GetTimerNanoseconds() < 3000000000ULL) {}

  LOG_INFO("Initialized debugging serial port and timing modules.");

  LOG_INFO("Initializing encoders...");
  InitEncoders();
  
  LOG_INFO("Initializing wheel speed estimator...");
  WheelStateEstimator::Init();

  LOG_INFO("Initializing robot state estimator...");
  InitRobotStateEstimator();

  LOG_INFO("Initializing inter-board communications...");
  Serial1.begin(1000000, SERIAL_8N1);

  LOG_INFO("Initializing motors...");
  InitMotors();

  LOG_INFO("Initializing servos...");
  InitServos();

  LOG_INFO("Register actions in action server...");
  p2p_action_server.Register(&sync_time_action_handler);
  p2p_action_server.Register(&set_head_pose_action_handler);
  p2p_action_server.Register(&set_base_velocity_action_handler);
  p2p_action_server.Register(&monitor_base_state_action_handler);

  LOG_INFO("Ready.");


  // --- Circle ---
  // // Walking straight around a circle.
  // const float kEnvelopeAmplitude = 0;
  // Happily walking around a circle.
  const float kEnvelopeAmplitude = 1;

  const int num_waypoints = base_carrier.capacity();
  const float total_trajectory_seconds = 20.0;
  for (int i = 0; i < num_waypoints; ++i) {
    const float t = i * total_trajectory_seconds / num_waypoints;
    const float w = 2 * M_PI / total_trajectory_seconds;
    const float x = sin(w * t);
    const float y = -1 + cos(w * t);
    base_carrier.Insert(BaseWaypoint(t, BaseTargetState({ BaseStateVars(Point(x, y), 0) })));
  }
  const auto base_carrier_view = BaseTrajectoryView(&base_carrier).EnableLooping(/*after_seconds=*/total_trajectory_seconds / num_waypoints).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear, .sampling_period_seconds = 0.1f });

  base_modulator.Insert(BaseWaypoint(0, BaseTargetState({ BaseStateVars(Point(0, 0), 0) })));
  base_modulator.Insert(BaseWaypoint(0.1, BaseTargetState({ BaseStateVars(Point(0, 0.015), 0) })));
  base_modulator.Insert(BaseWaypoint(0.3, BaseTargetState({ BaseStateVars(Point(0, -0.015), 0) })));
  const auto base_modulator_view = BaseTrajectoryView(&base_modulator).EnableLooping(/*after_seconds=*/0.1).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear, .sampling_period_seconds = 0.1f });
  
  base_envelope.Insert(EnvelopeWaypoint(0, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  base_envelope.Insert(EnvelopeWaypoint(0.5, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  base_envelope.Insert(EnvelopeWaypoint(3.5, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  const auto base_envelope_view = EnvelopeTrajectoryView(&base_envelope).EnableLooping(/*after_seconds=*/0.5).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear, .sampling_period_seconds = 0.1f });

  const BaseModulatedTrajectoryView base_trajectory_view(base_carrier_view, base_modulator_view, base_envelope_view);

  // // --- Square ---
  // // Walking straight around a square.
  // const float kEnvelopeAmplitude = 0;
  // // // Happily walking around a square.
  // // const float kEnvelopeAmplitude = 1;

  // const int num_waypoints = 4;
  // waypoints[0] = BaseWaypoint(0, BaseTargetState({ BaseStateVars(Point(0, 0), 0) }));
  // waypoints[1] = BaseWaypoint(4, BaseTargetState({ BaseStateVars(Point(1, 0), 0) }));
  // waypoints[2] = BaseWaypoint(8, BaseTargetState({ BaseStateVars(Point(1, -1), 0) }));
  // waypoints[3] = BaseWaypoint(12, BaseTargetState({ BaseStateVars(Point(0, -1), 0) }));
  // const auto carrier = BaseTrajectoryView(num_waypoints, waypoints).EnableLooping(/*after_seconds=*/4).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear, .sampling_period_seconds = 0.1f });

  // waypoints[num_waypoints] = BaseWaypoint(0.25, BaseTargetState({ BaseStateVars(Point(0, 0), 0) }));
  // waypoints[num_waypoints + 1] = BaseWaypoint(0.5, BaseTargetState({ BaseStateVars(Point(0, 0.03), 0) }));
  // waypoints[num_waypoints + 2] = BaseWaypoint(1, BaseTargetState({ BaseStateVars(Point(0, -0.03), 0) }));
  // const auto modulator = BaseTrajectoryView(3, &waypoints[num_waypoints]).EnableLooping(/*after_seconds=*/0.25).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear, .sampling_period_seconds = 0.1f });
  
  // envelope_waypoints[0] = EnvelopeWaypoint(0, EnvelopeTargetState({ EnvelopeStateVars(0 * kEnvelopeAmplitude) }));
  // envelope_waypoints[1] = EnvelopeWaypoint(0.5, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) }));
  // envelope_waypoints[2] = EnvelopeWaypoint(3.5, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) }));
  // const auto envelope = EnvelopeTrajectoryView(3, envelope_waypoints).EnableLooping(/*after_seconds=*/0.5).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear, .sampling_period_seconds = 0.1f });
  // const BaseModulatedTrajectoryView trajectory(carrier, modulator, envelope);

  base_trajectory_controller.trajectory(base_trajectory_view);
  base_trajectory_controller.StartTrajectory();


  head_carrier.Insert(HeadWaypoint(0, HeadTargetState({ HeadStateVars(0, 0) })));
  head_carrier.Insert(HeadWaypoint(0.2, HeadTargetState({ HeadStateVars(0, 0) })));
  head_carrier.Insert(HeadWaypoint(0.4, HeadTargetState({ HeadStateVars(0, 0) })));
  head_carrier.Insert(HeadWaypoint(0.8, HeadTargetState({ HeadStateVars(0, 0) })));
  head_carrier.Insert(HeadWaypoint(1, HeadTargetState({ HeadStateVars(0, 0) })));
  const auto head_carrier_view = HeadTrajectoryView(&head_carrier).EnableLooping(/*after_seconds=*/3).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear, .sampling_period_seconds = 0.1f });

  head_modulator.Insert(HeadWaypoint(0, HeadTargetState({ HeadStateVars(0, 0) })));
  head_modulator.Insert(HeadWaypoint(0.5, HeadTargetState({ HeadStateVars(0, M_PI / 8) })));
  head_modulator.Insert(HeadWaypoint(1.5, HeadTargetState({ HeadStateVars(0, -M_PI / 8) })));
  const auto head_modulator_view = HeadTrajectoryView(&head_modulator).EnableLooping(/*after_seconds=*/0.5).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear, .sampling_period_seconds = 0.1f });

  head_envelope.Insert(EnvelopeWaypoint(0, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  head_envelope.Insert(EnvelopeWaypoint(0.2, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  head_envelope.Insert(EnvelopeWaypoint(0.4, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  const auto head_envelope_view = EnvelopeTrajectoryView(&head_envelope).EnableLooping(/*after_seconds=*/0.2).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear, .sampling_period_seconds = 0.1f });

  const HeadModulatedTrajectoryView head_trajectory_view(head_carrier_view, head_modulator_view, head_envelope_view);
  
  head_trajectory_controller.trajectory(head_trajectory_view);
  head_trajectory_controller.StartTrajectory();
}

void loop() {
  wheel_state_estimator.Run();
  RunRobotStateEstimator();

  head_trajectory_controller.Run();
  base_trajectory_controller.Run();
  NotifyLeftMotorDirection(GetTimerTicks(), !base_trajectory_controller.base_speed_controller().left_wheel_speed_controller().is_turning_forward());
  NotifyRightMotorDirection(GetTimerTicks(), !base_trajectory_controller.base_speed_controller().right_wheel_speed_controller().is_turning_forward());
  left_wheel.Run();
  right_wheel.Run();
  
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

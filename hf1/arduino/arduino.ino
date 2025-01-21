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
#include "trajectory_store.h"
#include "create_base_trajectory_action_handler.h"
#include "create_head_trajectory_action_handler.h"
#include "create_envelope_trajectory_action_handler.h"
#include "create_base_trajectory_view_action_handler.h"
#include "create_head_trajectory_view_action_handler.h"
#include "create_envelope_trajectory_view_action_handler.h"
#include "create_base_modulated_trajectory_view_action_handler.h"
#include "create_head_modulated_trajectory_view_action_handler.h"
#include "create_base_mixed_trajectory_view_action_handler.h"
#include "create_head_mixed_trajectory_view_action_handler.h"
#include "execute_head_trajectory_view_action_handler.h"

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

TrajectoryStore trajectory_store;

P2PActionServer p2p_action_server(&p2p_stream);
SetHeadPoseActionHandler set_head_pose_action_handler(&p2p_stream);
SetBaseVelocityActionHandler set_base_velocity_action_handler(&p2p_stream, &base_speed_controller);
SyncTimeActionHandler sync_time_action_handler(&p2p_stream, &timer);
MonitorBaseStateActionHandler monitor_base_state_action_handler(&p2p_stream, &timer);
CreateBaseTrajectoryActionHandler create_base_trajectory_action_handler(&p2p_stream, &trajectory_store);
CreateHeadTrajectoryActionHandler create_head_trajectory_action_handler(&p2p_stream, &trajectory_store);
CreateEnvelopeTrajectoryActionHandler create_envelope_trajectory_action_handler(&p2p_stream, &trajectory_store);
CreateBaseTrajectoryViewActionHandler create_base_trajectory_view_action_handler(&p2p_stream, &trajectory_store);
CreateHeadTrajectoryViewActionHandler create_head_trajectory_view_action_handler(&p2p_stream, &trajectory_store);
CreateEnvelopeTrajectoryViewActionHandler create_envelope_trajectory_view_action_handler(&p2p_stream, &trajectory_store);
CreateBaseModulatedTrajectoryViewActionHandler create_base_modulated_trajectory_view_action_handler(&p2p_stream, &trajectory_store);
CreateHeadModulatedTrajectoryViewActionHandler create_head_modulated_trajectory_view_action_handler(&p2p_stream, &trajectory_store);
CreateBaseMixedTrajectoryViewActionHandler create_base_mixed_trajectory_view_action_handler(&p2p_stream, &trajectory_store);
CreateHeadMixedTrajectoryViewActionHandler create_head_mixed_trajectory_view_action_handler(&p2p_stream, &trajectory_store);
ExecuteHeadTrajectoryViewActionHandler execute_head_trajectory_view_action_handler(&p2p_stream, &trajectory_store, &head_trajectory_controller);

Trajectory<BaseTargetState, 40> base_carrier;
Trajectory<BaseTargetState, 10> base_modulator;
Trajectory<EnvelopeTargetState, 10> base_envelope;
BaseTrajectoryView base_carrier_view;
BaseTrajectoryView base_modulator_view;
EnvelopeTrajectoryView base_envelope_view;

Trajectory<HeadTargetState, 10> head_carrier;
Trajectory<HeadTargetState, 10> head_modulator;
Trajectory<EnvelopeTargetState, 10> head_envelope;
HeadTrajectoryView head_carrier_view;
HeadTrajectoryView head_modulator_view;
EnvelopeTrajectoryView head_envelope_view;

BaseModulatedTrajectoryView base_trajectory_view;
HeadModulatedTrajectoryView head_trajectory_view;

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
  p2p_action_server.Register(&create_base_trajectory_action_handler);
  p2p_action_server.Register(&create_head_trajectory_action_handler);
  p2p_action_server.Register(&create_envelope_trajectory_action_handler);
  p2p_action_server.Register(&create_base_trajectory_view_action_handler);
  p2p_action_server.Register(&create_head_trajectory_view_action_handler);
  p2p_action_server.Register(&create_envelope_trajectory_view_action_handler);
  p2p_action_server.Register(&create_base_modulated_trajectory_view_action_handler);
  p2p_action_server.Register(&create_head_modulated_trajectory_view_action_handler);
  p2p_action_server.Register(&create_base_mixed_trajectory_view_action_handler);
  p2p_action_server.Register(&create_head_mixed_trajectory_view_action_handler);
  p2p_action_server.Register(&execute_head_trajectory_view_action_handler);

  LOG_INFO("Ready.");

  left_wheel.Start();
  right_wheel.Start();
  
  // // --- Circle ---
  // // // Walking straight around a circle.
  // // const float kEnvelopeAmplitude = 0;
  // // Emotionally walking around a circle.
  // const float kEnvelopeAmplitude = 1;

  // const int num_waypoints = base_carrier.capacity();
  // const float total_trajectory_seconds = 20.0;
  // for (int i = 0; i < num_waypoints; ++i) {
  //   const float t = i * total_trajectory_seconds / num_waypoints;
  //   const float w = 2 * M_PI / total_trajectory_seconds;
  //   const float x = sin(w * t);
  //   const float y = -1 + cos(w * t);
  //   base_carrier.Insert(BaseWaypoint(t, BaseTargetState({ BaseStateVars(Point(x, y), 0) })));
  // }
  // const auto base_carrier_view = BaseTrajectoryView(&base_carrier).EnableLooping(/*after_seconds=*/total_trajectory_seconds / num_waypoints).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kCubic });

  // base_modulator.Insert(BaseWaypoint(0, BaseTargetState({ BaseStateVars(Point(0, 0), 0) })));
  // base_modulator.Insert(BaseWaypoint(0.1, BaseTargetState({ BaseStateVars(Point(0, 0.01), 0) })));
  // base_modulator.Insert(BaseWaypoint(0.3, BaseTargetState({ BaseStateVars(Point(0, -0.01), 0) })));
  // const auto base_modulator_view = BaseTrajectoryView(&base_modulator).EnableLooping(/*after_seconds=*/0.1).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kCubic });
  
  // base_envelope.Insert(EnvelopeWaypoint(0, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  // base_envelope.Insert(EnvelopeWaypoint(0.5, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  // base_envelope.Insert(EnvelopeWaypoint(3.5, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  // const auto base_envelope_view = EnvelopeTrajectoryView(&base_envelope).EnableLooping(/*after_seconds=*/0.5).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });

  // base_trajectory_view = BaseModulatedTrajectoryView(&base_carrier_view, &base_modulator_view, &base_envelope_view);

  // --- Square ---
  // // Walking straight around a square.
  // const float kEnvelopeAmplitude = 0;
  // Emotionally walking around a square.
  const float kEnvelopeAmplitude = 1;

  // Happy time increment.
  const float t_inc = kEnvelopeAmplitude * 1;
  // // Sad time increment.
  // const float t_inc = kEnvelopeAmplitude * 3;
  base_carrier.Insert(BaseWaypoint(0, BaseTargetState({ BaseStateVars(Point(0, 0), 0) })));
  base_carrier.Insert(BaseWaypoint(4 + t_inc, BaseTargetState({ BaseStateVars(Point(1, 0), 0) })));
  base_carrier.Insert(BaseWaypoint(8 + 2 * t_inc, BaseTargetState({ BaseStateVars(Point(1, -1), 0) })));
  base_carrier.Insert(BaseWaypoint(12 + 3 * t_inc, BaseTargetState({ BaseStateVars(Point(0, -1), 0) })));
  base_carrier_view = BaseTrajectoryView(&base_carrier).EnableLooping(/*after_seconds=*/4 + t_inc).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });

  // Happy modulator.
  base_modulator.Insert(BaseWaypoint(0, BaseTargetState({ BaseStateVars(Point(0, 0), 0) })));
  base_modulator.Insert(BaseWaypoint(0.1, BaseTargetState({ BaseStateVars(Point(0, 0.01), 0) })));
  base_modulator.Insert(BaseWaypoint(0.3, BaseTargetState({ BaseStateVars(Point(0, -0.01), 0) })));
  base_modulator_view = BaseTrajectoryView(&base_modulator).EnableLooping(/*after_seconds=*/0.1).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });

  // // Sad modulator.
  // base_modulator.Insert(BaseWaypoint(0, BaseTargetState({ BaseStateVars(Point(0, 0), 0) })));
  // base_modulator.Insert(BaseWaypoint(0.5, BaseTargetState({ BaseStateVars(Point(-0.01, 0), 0) })));
  // base_modulator.Insert(BaseWaypoint(0.7, BaseTargetState({ BaseStateVars(Point(0, 0), 0) })));
  // base_modulator_view = BaseTrajectoryView(&base_modulator).EnableLooping(/*after_seconds=*/0.5).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });
  
  base_envelope.Insert(EnvelopeWaypoint(0, EnvelopeTargetState({ EnvelopeStateVars(0 * kEnvelopeAmplitude) })));
  base_envelope.Insert(EnvelopeWaypoint(1, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  base_envelope.Insert(EnvelopeWaypoint(3 + t_inc, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  base_envelope_view = EnvelopeTrajectoryView(&base_envelope).EnableLooping(/*after_seconds=*/1).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });

  base_trajectory_view.carrier(&base_carrier_view).modulator(&base_modulator_view).envelope(&base_envelope_view);

  
  base_trajectory_controller.trajectory(&base_trajectory_view);
  // base_trajectory_controller.Start();


  // Happy carrier.
  const float head_down_seconds = (4 + t_inc) * 0.7;
  head_carrier.Insert(HeadWaypoint(head_down_seconds, HeadTargetState({ HeadStateVars(0, 0) })));
  head_carrier.Insert(HeadWaypoint(head_down_seconds + 0.2, HeadTargetState({ HeadStateVars(M_PI / 6, 0) })));
  head_carrier.Insert(HeadWaypoint(head_down_seconds + 0.8, HeadTargetState({ HeadStateVars(M_PI / 6, 0) })));
  head_carrier.Insert(HeadWaypoint(head_down_seconds + 1, HeadTargetState({ HeadStateVars(0, 0) })));
  head_carrier_view = HeadTrajectoryView(&head_carrier).EnableLooping(/*after_seconds=*/4 + t_inc).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });

  // // Sad carrier.
  // const float head_down_seconds = (4 + t_inc) * 0.7;
  // head_carrier.Insert(HeadWaypoint(head_down_seconds, HeadTargetState({ HeadStateVars(M_PI / 6, 0) })));
  // head_carrier.Insert(HeadWaypoint(head_down_seconds + 0.2, HeadTargetState({ HeadStateVars(0, 0) })));
  // head_carrier.Insert(HeadWaypoint(head_down_seconds + 0.8, HeadTargetState({ HeadStateVars(0, 0) })));
  // head_carrier.Insert(HeadWaypoint(head_down_seconds + 1, HeadTargetState({ HeadStateVars(M_PI / 6, 0) })));
  // head_carrier_view = HeadTrajectoryView(&head_carrier).EnableLooping(/*after_seconds=*/4 + t_inc).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });

  // Happy modulator.
  head_modulator.Insert(HeadWaypoint(0, HeadTargetState({ HeadStateVars(0, 0) })));
  head_modulator.Insert(HeadWaypoint(0.1, HeadTargetState({ HeadStateVars(0, M_PI / 16) })));
  head_modulator.Insert(HeadWaypoint(0.3, HeadTargetState({ HeadStateVars(0, -M_PI / 16) })));
  head_modulator_view = HeadTrajectoryView(&head_modulator).EnableLooping(/*after_seconds=*/0.1).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kCubic });

  // // Sad modulator.
  // head_modulator.Insert(HeadWaypoint(0, HeadTargetState({ HeadStateVars(0, 0) })));
  // head_modulator.Insert(HeadWaypoint(2, HeadTargetState({ HeadStateVars(0, M_PI / 8) })));
  // head_modulator.Insert(HeadWaypoint(4, HeadTargetState({ HeadStateVars(0, -M_PI / 16) })));
  // head_modulator_view = HeadTrajectoryView(&head_modulator).EnableLooping(/*after_seconds=*/1).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kCubic });

  head_envelope.Insert(EnvelopeWaypoint(0, EnvelopeTargetState({ EnvelopeStateVars(0 * kEnvelopeAmplitude) })));
  head_envelope.Insert(EnvelopeWaypoint(1, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  head_envelope.Insert(EnvelopeWaypoint(3 + t_inc, EnvelopeTargetState({ EnvelopeStateVars(1 * kEnvelopeAmplitude) })));
  head_envelope_view = EnvelopeTrajectoryView(&head_envelope).EnableLooping(/*after_seconds=*/1).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });

  head_trajectory_view.carrier(&head_carrier_view).modulator(&head_modulator_view).envelope(&head_envelope_view);
  
  head_trajectory_controller.trajectory(&head_trajectory_view);
  // head_trajectory_controller.Start();
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

// This program assumes the CPU clock is set to 96 MHz.

#include "utils.h"
#include "p2p_byte_stream_arduino.h"
#include "p2p_packet_stream.h"
#include "base_imu.h"
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
#include "execute_base_trajectory_view_action_handler.h"
#include "execute_head_trajectory_view_action_handler.h"
#include "console.h"
#include "operation_mode.h"
#include "led.h"
#include "led_controller.h"
#include "color_trajectory.h"
#include "persistance_offsets.h"
#include "persistance.h"

// Maximum time during which communication can be processed without
// yielding time to other tasks.
// This should be the minium period of all control loops.
#define kMaxRxTxLoopBlockingDurationNs 5'000'000

PersistentStorage persistent_storage(&Wire);

LedRed red_led;
LedGreen green_led;
LedBlue blue_led;
LedModulator led_modulator(&red_led, &green_led, &blue_led);
LedRGB rgb_led(&red_led, &green_led, &blue_led);
LedHSVTrajectoryController led_controller("LedController", &rgb_led);

Logger logger(&rgb_led);

// Trajectory<ColorHSVTargetState, /*Capacity=*/6> color_carrier_waypoints({
//   ColorHSVWaypoint(0, ColorHSVTargetState{{ ColorHSV(0, 1, 1) }}),
//   ColorHSVWaypoint(3, ColorHSVTargetState{{ ColorHSV(1/6.0, 1, 1) }}),
//   ColorHSVWaypoint(6, ColorHSVTargetState{{ ColorHSV(2/6.0, 1, 1) }}),
//   ColorHSVWaypoint(9, ColorHSVTargetState{{ ColorHSV(3/6.0, 1, 1) }}),
//   ColorHSVWaypoint(12, ColorHSVTargetState{{ ColorHSV(4/6.0, 1, 1) }}),
//   ColorHSVWaypoint(15, ColorHSVTargetState{{ ColorHSV(5/6.0, 1, 1) }}),
// });
// ColorHSVTrajectoryView color_carrier(&color_carrier_waypoints);
// Trajectory<ColorHSVTargetState, /*Capacity=*/2> color_modulator_waypoints({
//   ColorHSVWaypoint(0, ColorHSVTargetState{{ ColorHSV(0, 0, 0) }}),
//   ColorHSVWaypoint(0.5, ColorHSVTargetState{{ ColorHSV(0, 0, -1) }}),
// });
// ColorHSVTrajectoryView color_modulator(&color_modulator_waypoints);
// Trajectory<EnvelopeTargetState, /*Capacity=*/2> color_envelope_waypoints({
//   EnvelopeWaypoint(0, EnvelopeTargetState({EnvelopeStateVars(0.6f)})),
//   EnvelopeWaypoint(18, EnvelopeTargetState({EnvelopeStateVars(0.6f)})),
// });
// EnvelopeTrajectoryView color_envelope(&color_envelope_waypoints);
// ColorHSVModulatedTrajectoryView color_trajectory;

P2PByteStreamArduino byte_stream(&Serial1);
TimerArduino timer;
GUIDFactory guid_factory;
P2PPacketStreamArduino p2p_stream(&byte_stream, &timer, guid_factory);

BaseIMU base_imu(kPersistanceOffsetBaseIMUCalibration);

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
ExecuteBaseTrajectoryViewActionHandler execute_base_trajectory_view_action_handler(&p2p_stream, &trajectory_store, &base_trajectory_controller);
ExecuteHeadTrajectoryViewActionHandler execute_head_trajectory_view_action_handler(&p2p_stream, &trajectory_store, &head_trajectory_controller);

Console console(&Serial);

Trajectory<BaseTargetState, 10> base_traj;
BaseTrajectoryView base_traj_view(&base_traj);

void WaitForSerial() {
  TimerNanosType start_time = GetTimerNanoseconds();
  while (GetTimerNanoseconds() - start_time < 3'500'000'000ULL) {}
}

void setup() {
  // No need to call Serial.begin() with USB port.

  // Add logger instance to front of logger list.
  *logger.base_logger() = SetLogger(&logger);

  InitLeds();
  InitTimer();

  // Light all three LED colors to make sure they work.
  rgb_led.SetRGB(1, 1, 1);

  // Serial starts working after some time. Wait, so we don't miss any log.
  WaitForSerial();

  EnableWheelControl(true);
  EnableTrajectoryControl(true);

  LOG_INFO("Initialized debugging serial port and timing modules.");

  LOG_INFO("Initializing I2C bus 0.");
  Wire.begin();
  Wire.setDefaultTimeout(100'000/* us */);

  LOG_INFO("Initializing persistent storage.");
  persistent_storage.Init();

  LOG_INFO("Initializing encoders.");
  InitEncoders();

  LOG_INFO("Initializing wheel speed estimator.");
  WheelStateEstimator::Init();

  LOG_INFO("Initializing robot state estimator.");
  InitRobotStateEstimator();

  LOG_INFO("Initializing inter-board communications.");
  Serial1.begin(1000000, SERIAL_8N1);

  LOG_INFO("Initializing motors.");
  InitMotors();

  LOG_INFO("Initializing servos.");
  InitServos();

  LOG_INFO("Registering actions in action server.");
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
  p2p_action_server.Register(&execute_base_trajectory_view_action_handler);
  p2p_action_server.Register(&execute_head_trajectory_view_action_handler);

  left_wheel.Start();
  right_wheel.Start();

  rgb_led.SetRGB(0, 1, 0);

  LOG_INFO("Ready.");

  // --- Square ---
  // base_traj.Insert(BaseWaypoint(1.62, BaseTargetState({ BaseStateVars(Point(0, 0), 0) })));
  // base_traj.Insert(BaseWaypoint(2*1.62, BaseTargetState({ BaseStateVars(Point(1, 0), 0) })));
  // base_traj.Insert(BaseWaypoint(3*1.62, BaseTargetState({ BaseStateVars(Point(1, -1), 0) })));
  // base_traj.Insert(BaseWaypoint(4*1.62, BaseTargetState({ BaseStateVars(Point(0, -1), 0) })));
  // base_traj_view.EnableLooping(/*after_seconds=*/1.62).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });

  // base_trajectory_controller.trajectory(&base_traj_view);
  // base_trajectory_controller.Start();

  // color_carrier.EnableLooping(/*after_seconds=*/3).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });
  // color_modulator.EnableLooping(/*after_seconds=*/0.5).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });
  // color_envelope.DisableLooping().EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });
  // color_trajectory.carrier(&color_carrier).modulator(&color_modulator).envelope(&color_envelope);
  // led_controller.trajectory(&color_trajectory);
  // led_controller.Start();
}

void loop() {
  led_controller.Run();

  wheel_state_estimator.Run();
  RunRobotStateEstimator();

  if (IsTrajectoryControlEnabled()) {
    head_trajectory_controller.Run();
    base_trajectory_controller.Run();
    NotifyLeftMotorDirection(GetTimerTicks(), !base_trajectory_controller.base_speed_controller().left_wheel_speed_controller().is_turning_forward());
    NotifyRightMotorDirection(GetTimerTicks(), !base_trajectory_controller.base_speed_controller().right_wheel_speed_controller().is_turning_forward());
  }
  if (IsWheelControlEnabled()) {
    left_wheel.Run();
    right_wheel.Run();
  }

  bool process_comms = true;
  const auto process_comms_start_time_ns = timer.GetLocalNanoseconds();
  while (process_comms) {
    // Keep processing communications while there are bytes read
    // or ready to send.
    process_comms = p2p_stream.input().Run() > 0;
    p2p_stream.output().Run();
    p2p_action_server.Run();
    process_comms = process_comms || p2p_stream.output().NumCommittedPackets() > 0;
    // If processing for too long, yield time to other tasks.
    process_comms = process_comms && (timer.GetLocalNanoseconds() - process_comms_start_time_ns > kMaxRxTxLoopBlockingDurationNs);
  }

  console.Run();
}

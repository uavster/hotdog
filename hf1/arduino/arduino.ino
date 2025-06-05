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
#include "execute_base_trajectory_view_action_handler.h"
#include "execute_head_trajectory_view_action_handler.h"
#include "console.h"
#include "operation_mode.h"
#include "led.h"

// Maximum time during which communication can be processed without
// yielding time to other tasks.
// This should be the minium period of all control loops.
#define kMaxRxTxLoopBlockingDurationNs 5'000'000

Logger logger;

LedGreen green_led;
LedModulator led_modulator(&green_led);
LedRGB rgb_led(&green_led);

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
ExecuteBaseTrajectoryViewActionHandler execute_base_trajectory_view_action_handler(&p2p_stream, &trajectory_store, &base_trajectory_controller);
ExecuteHeadTrajectoryViewActionHandler execute_head_trajectory_view_action_handler(&p2p_stream, &trajectory_store, &head_trajectory_controller);

Console console(&Serial);

Trajectory<BaseTargetState, 10> base_traj;
BaseTrajectoryView base_traj_view(&base_traj);

class LedAnimation : public PeriodicRunnable {
public:
  LedAnimation() : PeriodicRunnable("led animation", static_cast<TimerSecondsType>(1.0/30)), level_(0) {}
  
  // Subclasses must override this function. It is called every given period.
  void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override {
    rgb_led.SetRGB(0, 1/32.0 + 1 - (1 + cosf(2 * M_PI * 1 * (SecondsFromNanos(now_nanos) - 5))) / 2, 0);
  }

private:
  uint8_t level_;
};

LedAnimation led_animation;

void setup() {  
  // No need to call Serial.begin() with USB port.

  // Configure logger.
  *logger.base_logger() = SetLogger(&logger);

  InitLeds();
  InitTimer();

  // Serial starts working after some time. Wait, so we don't miss any log.
  while(GetTimerNanoseconds() < 3'000'000'000ULL) {}

  EnableWheelControl(true);
  EnableTrajectoryControl(true);

  LOG_INFO("Initialized debugging serial port and timing modules.");

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

  LOG_INFO("Ready.");

// --- Square ---
// base_traj.Insert(BaseWaypoint(1.62, BaseTargetState({ BaseStateVars(Point(0, 0), 0) })));
// base_traj.Insert(BaseWaypoint(2*1.62, BaseTargetState({ BaseStateVars(Point(1, 0), 0) })));
// base_traj.Insert(BaseWaypoint(3*1.62, BaseTargetState({ BaseStateVars(Point(1, -1), 0) })));
// base_traj.Insert(BaseWaypoint(4*1.62, BaseTargetState({ BaseStateVars(Point(0, -1), 0) })));
// base_traj_view.EnableLooping(/*after_seconds=*/1.62).EnableInterpolation(InterpolationConfig{ .type = InterpolationType::kLinear });

// base_trajectory_controller.trajectory(&base_traj_view);
// base_trajectory_controller.Start();
}

void loop() {
  led_animation.Run();

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

  console.Run();
}

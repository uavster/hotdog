#ifndef ROBOT_SPEED_CONTROLLER_
#define ROBOT_SPEED_CONTROLLER_

#include "wheel_controller.h"
#include "periodic_runnable.h"
#include "robot_state.h"

// This is a closed-loop controller commanding wheel speed inner control loops to achieve
// the desired forward and angular speeds of the robot's base.
class BaseSpeedController {
public:
  // Does not take ownership of any pointee.
  BaseSpeedController(WheelSpeedController *left_wheel, WheelSpeedController *right_wheel);

  void SetTargetSpeeds(float linear, float angular);
  void Run();

private:
  WheelSpeedController &left_wheel_;
  WheelSpeedController &right_wheel_;
};

class BaseStateController : public PeriodicRunnable {
public:
  BaseStateController(BaseSpeedController *base_speed_controller);

  void SetTargetState(const Point &center_position_target, float yaw_target, float reference_forward_speed, float max_forward_speed, float reference_angular_speed = 0.0f);

  void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override;
  void Run();

private:
  BaseSpeedController &base_speed_controller_;
  Point center_position_target_;
  float yaw_target_;
  float reference_forward_speed_;
  float reference_angular_speed_;
  float max_forward_speed_;
};

#endif  // ROBOT_SPEED_CONTROLLER_
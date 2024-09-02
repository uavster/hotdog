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

  // Sets the target linear and angular speeds. The speeds might not be attainable by the 
  // wheels. If the linear speed is not attainable, the target is clamped to the wheel limit. 
  // Then, if the angular speed is not attainable, it is clamped to the angular speed limit 
  // given the updated target linear speed, and the linear speed is adjusted to keep the 
  // radius given by the updated linear speed and the original angular speed.
  void SetTargetSpeeds(float linear, float angular);

  // Returns the target linear speed adjusted to what the wheels can do.
  float target_linear_speed() const { return target_speed_linear_; }
  // Returns the target angular speed adjusted to what the wheels can do.
  float target_angular_speed() const { return target_speed_angular_; }
  // Returns the radius of the curve given the adjusted target speeds.
  float curve_radius() const { return target_linear_speed() / target_angular_speed(); }

  void Run();

  const WheelSpeedController &left_wheel_speed_controller() const { return left_wheel_; }
  const WheelSpeedController &right_wheel_speed_controller() const { return right_wheel_; }

private:
  WheelSpeedController &left_wheel_;
  WheelSpeedController &right_wheel_;
  float target_speed_linear_;
  float target_speed_angular_;
};

class BaseStateController : public PeriodicRunnable {
public:
  BaseStateController(BaseSpeedController *base_speed_controller);

  void SetTargetState(const Point &center_position_target, float yaw_target, float reference_forward_speed, float reference_angular_speed = 0.0f);

  void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override;
  void Run();

  const BaseSpeedController &base_speed_controller() const { return base_speed_controller_; }

  bool IsAtTargetState() const;

private:
  BaseSpeedController &base_speed_controller_;
  Point center_position_target_;
  float yaw_target_;
  float reference_forward_speed_;
  float reference_angular_speed_;
};

#endif  // ROBOT_SPEED_CONTROLLER_
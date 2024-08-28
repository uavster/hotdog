#include <algorithm>
#include "base_controller.h"
#include "utils.h"
#include "robot_model.h"
#include "robot_state_estimator.h"

#define kLoopPeriod 0.33  // [s]

#define kKx 10.0   // [1/s]
#define kKy 64.0 // [1/m^2]
#define kKyaw 16.0    // [1/m]

BaseSpeedController::BaseSpeedController(WheelSpeedController *left_wheel, WheelSpeedController *right_wheel) 
  : left_wheel_(*ASSERT_NOT_NULL(left_wheel)), right_wheel_(*ASSERT_NOT_NULL(right_wheel)) {
  ASSERT(left_wheel_.GetMaxAngularSpeed() == right_wheel_.GetMaxAngularSpeed());
}

void BaseSpeedController::SetTargetSpeeds(const float linear, const float angular) {
  target_speed_linear_ = linear;
  target_speed_angular_ = angular;
  if (linear >= 0) {
    target_speed_linear_ = min(target_speed_linear_, left_wheel_.GetMaxLinearSpeed());
    if (angular >= 0) {
      const float angular_max = 2 * (left_wheel_.GetMaxLinearSpeed() - target_speed_linear_) / kRobotDistanceBetweenTireCenters;
      target_speed_angular_ = min(angular_max, target_speed_angular_);
    } else {
      const float angular_min = -2 * (left_wheel_.GetMaxLinearSpeed() - target_speed_linear_) / kRobotDistanceBetweenTireCenters;
      target_speed_angular_ = max(angular_min, target_speed_angular_);
    }
  } else {
    target_speed_linear_ = max(target_speed_linear_, left_wheel_.GetMinLinearSpeed());
    if (angular >= 0) {
      const float angular_max = -2 * (left_wheel_.GetMinLinearSpeed() - target_speed_linear_) / kRobotDistanceBetweenTireCenters;
      target_speed_angular_ = min(angular_max, target_speed_angular_);
    } else {
      const float angular_min = 2 * (left_wheel_.GetMinLinearSpeed() - target_speed_linear_) / kRobotDistanceBetweenTireCenters;
      target_speed_angular_ = max(angular_min, target_speed_angular_);
    }
  }
  if (angular != target_speed_angular_) {
    // Prevents NaN linear speed when angular speed is zero.
    target_speed_linear_ = (target_speed_linear_ * target_speed_angular_) / angular;
  }
  
  const float linear_component = target_speed_linear_ / kWheelRadius;
  const float angular_component = (target_speed_angular_ * kRobotDistanceBetweenTireCenters) / (2 * kWheelRadius);
  const float wheel_speed_l = linear_component - angular_component;
  const float wheel_speed_r = linear_component + angular_component;

  // Serial.printf("l:%f r:%f\n", wheel_speed_l, wheel_speed_r);

  left_wheel_.SetAngularSpeed(wheel_speed_l);
  right_wheel_.SetAngularSpeed(wheel_speed_r);
}

void BaseSpeedController::Run() {
  left_wheel_.Run();
  right_wheel_.Run();
}

BaseStateController::BaseStateController(BaseSpeedController *base_speed_controller) 
  : PeriodicRunnable(kLoopPeriod), 
    base_speed_controller_(*ASSERT_NOT_NULL(base_speed_controller)),
    yaw_target_(0),
    reference_forward_speed_(0),
    reference_angular_speed_(0) {}

void BaseStateController::Run() {
  base_speed_controller_.Run();
  PeriodicRunnable::Run();
}

void BaseStateController::RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) {
  // Get errors in the base's local frame.
  const BaseState &base_state = GetBaseState();
  const float cos_yaw = cos(base_state.yaw());
  const float sin_yaw = sin(base_state.yaw());
  const Point position_error = center_position_target_ - base_state.center();
  const float forward_error = position_error.x * cos_yaw + position_error.y * sin_yaw;
  const float lateral_error = -position_error.x * sin_yaw + position_error.y * cos_yaw;  
  const float yaw_error = yaw_target_ - base_state.yaw();

  // Calculate commands for the base's speed controller from errors and reference speed.
  const float linear_speed_command = reference_forward_speed_ * cos(yaw_error) + kKx * forward_error;
  // Reference angular speed is assumed to be 0.
  const float angular_speed_command = reference_forward_speed_ * (kKy * lateral_error + kKyaw * sin(yaw_error));
  Serial.printf("forward_cmd:%f angular_cmd:%f\n", min(0.4, max(0, linear_speed_command)), min(0.8, max(-0.8, angular_speed_command)));
  base_speed_controller_.SetTargetSpeeds(min(0.4, max(0, linear_speed_command)), min(0.8, max(-0.8, angular_speed_command)));
}

void BaseStateController::SetTargetState(const Point &center_position_target, float yaw_target, float reference_forward_speed, float reference_angular_speed) {
  center_position_target_ = center_position_target;
  yaw_target_ = yaw_target;
  reference_forward_speed_ = reference_forward_speed;
  reference_angular_speed_ = reference_angular_speed;
}


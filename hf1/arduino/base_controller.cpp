#include <iterator>
#include <algorithm>
#include "base_controller.h"
#include "utils.h"
#include "logger_interface.h"
#include "robot_model.h"
#include "robot_state_estimator.h"

#define kBaseStateControllerLoopPeriod 0.33  // [s]
#define kBaseTrajectoryControllerLoopPeriod 0.1 // [s]

#define kKx 3.0   // [1/s]
#define kKy 64.0 // [1/m^2]
#define kKyaw 32.0    // [1/m]

#define kBaseTrajectoryControllerK1 0.8
#define kBaseTrajectoryControllerK2 3.0*3
#define kBaseTrajectoryControllerK3 0.8

#define kBaseTrajectoryTangentialSpeedMax 0.4
#define kBaseTrajectoryAngularSpeedMax (2*M_PI/3)

#define kMaxPositionErrorForAtState 0.15 // [m]
#define kMaxYawErrorForAtState ((45.0/4 * M_PI) / 180)  // [rad]

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

BaseStateController::BaseStateController(BaseSpeedController *base_speed_controller) 
  : Controller(kBaseStateControllerLoopPeriod), 
    base_speed_controller_(*ASSERT_NOT_NULL(base_speed_controller)),
    yaw_target_(0),
    reference_forward_speed_(0),
    reference_angular_speed_(0) {}

void BaseStateController::Update(TimerSecondsType now_seconds) {
  // Get errors in the base's local frame.
  const BaseState &base_state = GetBaseState();
  const float cos_yaw = cos(base_state.location().yaw());
  const float sin_yaw = sin(base_state.location().yaw());
  const Point position_error = center_position_target_ - base_state.location().position();
  const float forward_error = position_error.x * cos_yaw + position_error.y * sin_yaw;
  const float lateral_error = -position_error.x * sin_yaw + position_error.y * cos_yaw;  
  const float yaw_error = NormalizeRadians(yaw_target_ - base_state.location().yaw());

  // Calculate commands for the base's speed controller from errors and reference speed.
  float linear_speed_command = reference_forward_speed_ * cos(yaw_error) + kKx * forward_error;
  linear_speed_command = std::clamp(linear_speed_command, -0.4, 0.4);
  // Reference angular speed is assumed to be 0.
  float angular_speed_command = reference_forward_speed_ * (kKy * lateral_error + kKyaw * sin(yaw_error));
  angular_speed_command = std::clamp(angular_speed_command, -0.8, 0.8);
  // Serial.printf("%f * (%f * %f + %f * sin(%f))\n", reference_forward_speed_, kKy, lateral_error, kKyaw, yaw_error);
  // Serial.printf("fw_error:%f lat_error:%f yaw_error:%f fw_cmd:%f ang_cmd:%f\n", forward_error, lateral_error, yaw_error, linear_speed_command, angular_speed_command);
  base_speed_controller_.SetTargetSpeeds(linear_speed_command, angular_speed_command);
}

void BaseStateController::SetTargetState(const Point &center_position_target, float yaw_target, float reference_forward_speed, float reference_angular_speed) {
  center_position_target_ = center_position_target;
  yaw_target_ = yaw_target;
  reference_forward_speed_ = reference_forward_speed;
  reference_angular_speed_ = reference_angular_speed;
}

bool BaseStateController::IsAtTargetState() const {
  const BaseState &base_state = GetBaseState();
  const Point position_error = center_position_target_ - base_state.location().position();
  const float yaw_error = NormalizeRadians(yaw_target_ - base_state.location().yaw());
  return position_error.norm() <= kMaxPositionErrorForAtState && abs(yaw_error) <= kMaxYawErrorForAtState;
}

BaseTrajectoryController::BaseTrajectoryController(BaseSpeedController *base_speed_controller) : 
  TrajectoryController<BaseTrajectoryView>(static_cast<TimerSecondsType>(kBaseTrajectoryControllerLoopPeriod)), 
  base_speed_controller_(*ASSERT_NOT_NULL(base_speed_controller)) {}

void BaseTrajectoryController::Update(TimerSecondsType seconds_since_start, int current_waypoint_index) {
  Serial.printf("current_waypoint_index: %d\n", current_waypoint_index);
  // Get reference states.
  TimerSecondsType time_fraction = (seconds_since_start - trajectory().seconds(current_waypoint_index)) / (trajectory().seconds(current_waypoint_index + 1) - trajectory().seconds(current_waypoint_index));
  const State ref_position = trajectory().state(current_waypoint_index) + time_fraction * (trajectory().state(current_waypoint_index + 1) - trajectory().state(current_waypoint_index));
  const State first_derivative_at_index = trajectory().derivative(/*order=*/1, current_waypoint_index);
  const State ref_velocity = first_derivative_at_index + time_fraction * (trajectory().derivative(/*order=*/1, current_waypoint_index + 1) - first_derivative_at_index);
  const State second_derivative_at_index = trajectory().derivative(/*order=*/2, current_waypoint_index);
  const State ref_acceleration = second_derivative_at_index + time_fraction * (trajectory().derivative(/*order=*/2, current_waypoint_index + 1) - second_derivative_at_index);
  const float ref_yaw = atan2f(ref_velocity.location().position().y, ref_velocity.location().position().x);

  // Serial.printf("[ref] t:%f t+1:%f x:%f y:%f vx:%f vy:%f ax:%f ay:%f\n", trajectory().seconds(current_waypoint_index), trajectory().seconds(current_waypoint_index+1), ref_position.location().position().x, ref_position.location().position().y, ref_velocity.location().position().x, ref_velocity.location().position().y, ref_acceleration.location().position().x, ref_acceleration.location().position().y);
  // Serial.printf("[state] x:%f y:%f\n", GetBaseState().location().position().x, GetBaseState().location().position().y);
  // Serial.printf("[controller] target_lin:%f target_ang:%f left_wheel_ang:%f right_wheel_ang:%f\n", base_speed_controller_.target_linear_speed(), base_speed_controller_.target_angular_speed(), base_speed_controller_.left_wheel_speed_controller().GetAngularSpeed(), base_speed_controller_.right_wheel_speed_controller().GetAngularSpeed());

  // Get errors in the base's local frame.
  const BaseState &base_state = GetBaseState();
  const float cos_yaw = cos(base_state.location().yaw());
  const float sin_yaw = sin(base_state.location().yaw());
  const Point position_error = ref_position.location().position() - base_state.location().position();
  const float forward_error = position_error.x * cos_yaw + position_error.y * sin_yaw;
  const float lateral_error = -position_error.x * sin_yaw + position_error.y * cos_yaw;  
  const float yaw_error = NormalizeRadians(ref_yaw - base_state.location().yaw());

  // Get feedforward velocities.
  const float feedfoward_tangential_velocity = ref_velocity.location().position().norm();
  if (abs(feedfoward_tangential_velocity) < 1e-6) {
    base_speed_controller_.SetTargetSpeeds(0, 0);
    return;
  }  
  const float feedfoward_angular_velocity = (ref_velocity.location().position().x * ref_acceleration.location().position().y - ref_velocity.location().position().y * ref_acceleration.location().position().x) / feedfoward_tangential_velocity;
  // Serial.printf("feedfoward_tangential_velocity:%f feedfoward_angular_velocity:%f\n", feedfoward_tangential_velocity, feedfoward_angular_velocity);
  // Get feedforward commands.
  const float feedforward_tangential_command = cos(yaw_error) * feedfoward_tangential_velocity;
  const float feedforward_angular_command = feedfoward_angular_velocity;

  // Get feedback commands.
  const float feedback_tangential_command = kBaseTrajectoryControllerK1 * forward_error;
  const float feedback_angular_command = kBaseTrajectoryControllerK2 * lateral_error + kBaseTrajectoryControllerK3 * yaw_error;

  // Get velocity commands.
  const float tangential_command = feedforward_tangential_command + feedback_tangential_command;
  const float angular_command = feedback_angular_command + feedforward_angular_command;
  const float tangential_command_limitted = std::clamp(tangential_command, -kBaseTrajectoryTangentialSpeedMax, kBaseTrajectoryTangentialSpeedMax);
  const float angular_command_limitted = std::clamp(angular_command, -kBaseTrajectoryAngularSpeedMax, kBaseTrajectoryAngularSpeedMax);

  // Serial.printf("%f %f\n", tangential_command_limitted, angular_command_limitted);
  base_speed_controller_.SetTargetSpeeds(tangential_command_limitted, angular_command_limitted);
}


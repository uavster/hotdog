#include <iterator>
#include <algorithm>
#include "base_controller.h"
#include "utils.h"
#include "logger_interface.h"
#include "robot_model.h"
#include "robot_state_estimator.h"

#define kBaseStateControllerLoopPeriod 0.33  // [s]
#define kBaseTrajectoryControllerLoopPeriod 0.03 // [s]

#define kKx 3.0   // [1/s]
#define kKy 64.0 // [1/m^2]
#define kKyaw 32.0    // [1/m]

#define kBaseTrajectoryControllerK1 0.8
#define kBaseTrajectoryControllerK2 3.0
#define kBaseTrajectoryControllerK3 0.8*2

#define kBaseTrajectoryTangentialSpeedMax 0.5
#define kBaseTrajectoryAngularSpeedMax (2*M_PI/3)

#define kMaxPositionErrorForAtState 0.15 // [m]
#define kMaxYawErrorForAtState ((45.0/4 * M_PI) / 180)  // [rad]

#include <Arduino.h>

BaseWaypoint BaseModulatedTrajectoryView::GetWaypoint(float seconds) const {
  // return carrier().GetWaypoint(index);
  // Modulate the carrier with the enveloped modulator.
  // Transform the modulator with the carrier.
  const auto carrier_pos = carrier().state(seconds).location().position();
  const auto carrier_pos_diff = carrier().state(seconds + kBaseTrajectoryControllerLoopPeriod).location().position() - carrier_pos;
  const float carrier_angle = -atan2f(carrier_pos_diff.y, carrier_pos_diff.x);
  const float cos_angle = cosf(carrier_angle);
  const float sin_angle = sinf(carrier_angle);
  const float envelope_value = envelope().state(seconds).location().amplitude();
  const auto modulator_pos = modulator().state(seconds).location().position() * envelope_value;
  return BaseWaypoint(
    /*seconds=*/seconds, 
    BaseTargetState({
      BaseStateVars(
        Point(
          modulator_pos.x * cos_angle + modulator_pos.y * sin_angle + carrier_pos.x, 
          modulator_pos.y * cos_angle - modulator_pos.x * sin_angle + carrier_pos.y
        ), 
        /*yaw=*/0
      )
    })
  );
}

BaseSpeedController::BaseSpeedController(WheelSpeedController *left_wheel, WheelSpeedController *right_wheel) 
  : left_wheel_(*ASSERT_NOT_NULL(left_wheel)), right_wheel_(*ASSERT_NOT_NULL(right_wheel)) {
  ASSERT(left_wheel_.GetMaxAngularSpeed() == right_wheel_.GetMaxAngularSpeed());
}

void BaseSpeedController::SetTargetSpeeds(const float linear, const float angular) {
  target_speed_linear_ = linear;
  target_speed_angular_ = angular;
  if (linear >= 0) {
    target_speed_linear_ = std::min(target_speed_linear_, left_wheel_.GetMaxLinearSpeed());
    if (angular >= 0) {
      const float angular_max = 2 * (left_wheel_.GetMaxLinearSpeed() - target_speed_linear_) / kRobotDistanceBetweenTireCenters;
      target_speed_angular_ = std::min(angular_max, target_speed_angular_);
    } else {
      const float angular_min = -2 * (left_wheel_.GetMaxLinearSpeed() - target_speed_linear_) / kRobotDistanceBetweenTireCenters;
      target_speed_angular_ = std::max(angular_min, target_speed_angular_);
    }
  } else {
    target_speed_linear_ = std::max(target_speed_linear_, left_wheel_.GetMinLinearSpeed());
    if (angular >= 0) {
      const float angular_max = -2 * (left_wheel_.GetMinLinearSpeed() - target_speed_linear_) / kRobotDistanceBetweenTireCenters;
      target_speed_angular_ = std::min(angular_max, target_speed_angular_);
    } else {
      const float angular_min = 2 * (left_wheel_.GetMinLinearSpeed() - target_speed_linear_) / kRobotDistanceBetweenTireCenters;
      target_speed_angular_ = std::max(angular_min, target_speed_angular_);
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

BaseStateController::BaseStateController(const char *name, BaseSpeedController *base_speed_controller) 
  : Controller(name, kBaseStateControllerLoopPeriod), 
    base_speed_controller_(*ASSERT_NOT_NULL(base_speed_controller)),
    yaw_target_(0),
    reference_forward_speed_(0),
    reference_angular_speed_(0) {}

void BaseStateController::StopControl() {
  base_speed_controller_.SetTargetSpeeds(0, 0);
}

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

BaseTrajectoryController::BaseTrajectoryController(const char *name, BaseSpeedController *base_speed_controller) : 
  TrajectoryController<BaseModulatedTrajectoryView>(name, static_cast<TimerSecondsType>(kBaseTrajectoryControllerLoopPeriod)), 
  base_speed_controller_(*ASSERT_NOT_NULL(base_speed_controller)) {}

void BaseTrajectoryController::Update(TimerSecondsType seconds_since_start) {
  TrajectoryController<BaseModulatedTrajectoryView>::Update(seconds_since_start);
  if (!is_started()) { return; }

  // Get reference states.
  const State ref_position = trajectory().state(seconds_since_start);
  const State ref_velocity = trajectory().derivative(/*order=*/1, seconds_since_start, /*epsilon=*/kBaseTrajectoryControllerLoopPeriod);
  const State ref_acceleration = trajectory().derivative(/*order=*/2, seconds_since_start, /*epsilon=*/kBaseTrajectoryControllerLoopPeriod);
  const float ref_yaw = atan2f(ref_velocity.location().position().y, ref_velocity.location().position().x);

  Serial.printf("ref %f: %f %f\n", seconds_since_start, ref_position.location().position().x, ref_position.location().position().y);

  // Get errors in the base's local frame.
  const BaseState &base_state = GetBaseState();
  const float cos_yaw = cos(base_state.location().yaw());
  const float sin_yaw = sin(base_state.location().yaw());
  const Point position_error = ref_position.location().position() - base_state.location().position();
  const float forward_error = position_error.x * cos_yaw + position_error.y * sin_yaw;
  const float lateral_error = -position_error.x * sin_yaw + position_error.y * cos_yaw;  
  const float yaw_error = NormalizeRadians(ref_yaw - base_state.location().yaw());

  Serial.printf("state: %f %f\n", base_state.location().position().x, base_state.location().position().y);

  float feedforward_tangential_command = 0;
  float feedforward_angular_command = 0;

  // Get feedforward velocities.
  const float feedfoward_tangential_velocity = ref_velocity.location().position().norm();
  if (abs(feedfoward_tangential_velocity) >= 1e-6) {
    const float feedfoward_angular_velocity = (ref_velocity.location().position().x * ref_acceleration.location().position().y - ref_velocity.location().position().y * ref_acceleration.location().position().x) / feedfoward_tangential_velocity;
    // Get feedforward commands.
    feedforward_tangential_command = cos(yaw_error) * feedfoward_tangential_velocity;
    feedforward_angular_command = feedfoward_angular_velocity;
  }

  // Get feedback commands.
  const float feedback_tangential_command = kBaseTrajectoryControllerK1 * forward_error;
  const float feedback_angular_command = kBaseTrajectoryControllerK2 * lateral_error + kBaseTrajectoryControllerK3 * yaw_error;

  // Get velocity commands.
  const float tangential_command = feedforward_tangential_command + feedback_tangential_command;
  const float angular_command = feedforward_angular_command + feedback_angular_command;
  const float tangential_command_limitted = std::clamp(tangential_command, -kBaseTrajectoryTangentialSpeedMax, kBaseTrajectoryTangentialSpeedMax);
  const float angular_command_limitted = std::clamp(angular_command, -kBaseTrajectoryAngularSpeedMax, kBaseTrajectoryAngularSpeedMax);

  Serial.printf("cmd: %f %f\n", tangential_command_limitted, angular_command_limitted);
  
  base_speed_controller_.SetTargetSpeeds(tangential_command_limitted, angular_command_limitted);
}

void BaseTrajectoryController::StopControl() {
  base_speed_controller_.SetTargetSpeeds(0, 0);
}

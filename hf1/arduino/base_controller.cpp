#include <iterator>
#include <algorithm>
#include "base_controller.h"
#include "utils.h"
#include "robot_model.h"
#include "robot_state_estimator.h"

#define kLoopPeriod 0.33  // [s]
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
  const float yaw_error = NormalizeRadians(yaw_target_ - base_state.yaw());

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
  const Point position_error = center_position_target_ - base_state.center();
  const float yaw_error = NormalizeRadians(yaw_target_ - base_state.yaw());
  return position_error.norm() <= kMaxPositionErrorForAtState && abs(yaw_error) <= kMaxYawErrorForAtState;
}

BaseTrajectoryView &BaseTrajectoryView::EnableLooping(TimerSecondsType after_seconds) { 
  if (num_waypoints_ == 0) { 
    loop_at_seconds_ = -1;
  } else {
    loop_at_seconds_ = waypoints_[num_waypoints_ - 1].seconds() + after_seconds;
  }
  return *this;
}

BaseTrajectoryView &BaseTrajectoryView::DisableLooping() { 
  loop_at_seconds_ = -1;
  return *this;
}

bool BaseTrajectoryView::IsLoopingEnabled() const { 
  return loop_at_seconds_ >= 0;
}

StatusOr<int> BaseTrajectoryView::FindWaypointIndexBeforeSeconds(TimerSecondsType seconds, int prev_result_index) const {
  if (num_waypoints_ == 0 || seconds < this->seconds(0)) { return Status::kUnavailableError; }
  int i = prev_result_index;
  while((IsLoopingEnabled() || i < num_waypoints_) && this->seconds(i) < seconds) { ++i; }
  return i - 1;
}

float BaseTrajectoryView::seconds(int index) const {
  if (IsLoopingEnabled()) {
    const int normalized_index = IndexMod(index, num_waypoints_);
    const TimerSecondsType prev_loops_seconds = (loop_at_seconds_ - waypoints_[0].seconds()) * (index / num_waypoints_);
    return waypoints_[normalized_index].seconds() + prev_loops_seconds;
  } else {
    ASSERT(index >= 0);
    ASSERT(index < num_waypoints_);
    return waypoints_[index].seconds();
  }
}

const Point &BaseTrajectoryView::position(int index) const {
  if (IsLoopingEnabled()) {
    return waypoints_[IndexMod(index, num_waypoints_)].position();
  } else {
    return waypoints_[min(index, num_waypoints_ - 1)].position();
  }
}

Point BaseTrajectoryView::velocity(int index) const {
  return (position(index + 1) - position(index)) / (seconds(index + 1) - seconds(index));
}

Point BaseTrajectoryView::acceleration(int index) const {
  return (velocity(index + 1) - velocity(index)) / (seconds(index + 1) - seconds(index));
}

BaseTrajectoryController::BaseTrajectoryController(BaseSpeedController *base_speed_controller) : 
  PeriodicRunnable(kBaseTrajectoryControllerLoopPeriod), 
  base_speed_controller_(*ASSERT_NOT_NULL(base_speed_controller)), 
  current_waypoint_index_(0), 
  is_started_(false),
  does_loop_(false) {}

void BaseTrajectoryController::trajectory(const BaseTrajectoryView &trajectory) {
  trajectory_ = trajectory;
  current_waypoint_index_ = 0;
  is_started_ = false;
}

void BaseTrajectoryController::Run() {
  PeriodicRunnable::Run();
  base_speed_controller_.Run();
}

void BaseTrajectoryController::StartTrajectory() {
  current_waypoint_index_ = 0;
  is_started_ = true;
  start_seconds_ = GetTimerSeconds();
}

void BaseTrajectoryController::StopTrajectory() {
  is_started_ = false;
}

void BaseTrajectoryController::RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) {
  if (!is_started_) { return; }
  const TimerSecondsType now_seconds = SecondsFromNanos(now_nanos) - start_seconds_;
  const auto maybe_index = trajectory_.FindWaypointIndexBeforeSeconds(now_seconds, current_waypoint_index_);
  if (!maybe_index.ok()) { return; }
  const int index = *maybe_index;
  current_waypoint_index_ = index;
  Serial.printf("index:%d\n", index);
  if (!trajectory_.IsLoopingEnabled() && index == trajectory_.num_waypoints() - 1) {
    StopTrajectory();
    return;
  }

  // Get reference states.
  TimerSecondsType time_fraction = (now_seconds - trajectory_.seconds(index)) / (trajectory_.seconds(index + 1) - trajectory_.seconds(index));
  const Point ref_position = trajectory_.position(index); + time_fraction * (trajectory_.position(index + 1) - trajectory_.position(index));
  const Point ref_velocity = trajectory_.velocity(index) + time_fraction * (trajectory_.velocity(index + 1) - trajectory_.velocity(index));
  const Point ref_acceleration = trajectory_.acceleration(index) + time_fraction * (trajectory_.acceleration(index + 1) - trajectory_.acceleration(index));
  const float ref_yaw = atan2f(ref_velocity.y, ref_velocity.x);

  Serial.printf("[ref] t:%f t+1:%f x:%f y:%f vx:%f vy:%f ax:%f ay:%f\n", trajectory_.seconds(index), trajectory_.seconds(index+1), ref_position.x, ref_position.y, ref_velocity.x, ref_velocity.y, ref_acceleration.x, ref_acceleration.y);
  Serial.printf("[state] x:%f y:%f\n", GetBaseState().center().x, GetBaseState().center().y);

  // Get errors in the base's local frame.
  const BaseState &base_state = GetBaseState();
  const float cos_yaw = cos(base_state.yaw());
  const float sin_yaw = sin(base_state.yaw());
  const Point position_error = ref_position - base_state.center();
  const float forward_error = position_error.x * cos_yaw + position_error.y * sin_yaw;
  const float lateral_error = -position_error.x * sin_yaw + position_error.y * cos_yaw;  
  const float yaw_error = NormalizeRadians(ref_yaw - base_state.yaw());

  // Get feedforward velocities.
  const float feedfoward_tangential_velocity = ref_velocity.norm();
  if (abs(feedfoward_tangential_velocity) < 1e-6) {
    base_speed_controller_.SetTargetSpeeds(0, 0);
    return;
  }  
  const float feedfoward_angular_velocity = (ref_velocity.x * ref_acceleration.y - ref_velocity.y * ref_acceleration.x) / feedfoward_tangential_velocity;
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


#include "robot_state.h"

RobotState::RobotState() : left_wheel_ticks_(0), right_wheel_ticks_(0), angle_(0.0), left_wheel_moving_backward_(false), right_wheel_moving_backward_(false) {}

void RobotState::NotifyWheelTicks(int left_ticks_inc, int right_ticks_inc) {
  if (left_wheel_moving_backward_) {
    left_ticks_inc = -left_ticks_inc;
  }
  if (right_wheel_moving_backward_) {
    right_ticks_inc = -right_ticks_inc;
  }
  left_wheel_ticks_ += left_ticks_inc;
  right_wheel_ticks_ += right_ticks_inc;
  float distance_inc = (kRadiansPerWheelTick * kWheelRadius * (left_ticks_inc + right_ticks_inc)) / 2;
  angle_ = ((kRadiansPerWheelTick * kWheelRadius) * (right_wheel_ticks_ - left_wheel_ticks_)) / kRobotDistanceBetweenTireCenters;
  center_.x += distance_inc * cos(angle_);
  center_.y += distance_inc * sin(angle_);
}

void RobotState::NotifyLeftWheelDirection(bool backward) {
  left_wheel_moving_backward_ = backward;
}

void RobotState::NotifyRightWheelDirection(bool backward) {
  right_wheel_moving_backward_ = backward;
}

const Point &RobotState::Center() {
  return center_;
}

const float RobotState::Angle() {
  return angle_;
}
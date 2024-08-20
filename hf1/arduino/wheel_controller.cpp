#include <limits>
#include "wheel_controller.h"
#include "robot_model.h"
#include "utils.h"

#define kControlLoopPeriodSeconds 1e-2

#define kSpeedModelTimeConstant 0.29
#define kSpeedModelDutyCycleOffset -0.99
#define kSpeedModelFactor 0.041
#define kSpeedModelSpeedOffset 0.66

// Best params for 0.6 m/s
// #define kP 20.0
// #define kI 50.0
// #define kD 0.0

// // Best params for 0.15 m/s
// #define kP 20.0
// #define kI 10.0
// #define kD 0.5

// Best params for 0.4 m/s
#define kP 10.0
#define kI 10.0
#define kD 0.5

// Any wheel speed estimate above this value is rejected.
// Meant to prevent too high estimates due to co-occuring encoder edges.
#define kWheelRadiansPerSecondMax (2.0 / kWheelRadius) // [rad/s]

int32_t left_wheel_num_ticks;
int32_t right_wheel_num_ticks;

static void LeftEncoderIsr(TimerTicksType timer_ticks) {
  ++left_wheel_num_ticks;
}

static void RightEncoderIsr(TimerTicksType timer_ticks) {
  ++right_wheel_num_ticks;
}

void InitWheelSpeedControl() {
  left_wheel_num_ticks = 0;
  right_wheel_num_ticks = 0;
  AddEncoderIsrs(&LeftEncoderIsr, &RightEncoderIsr);
}

int32_t GetLeftWheelTickCount() {
  NO_ENCODER_IRQ {
    return left_wheel_num_ticks;
  }
  return 0;
}

int32_t GetRightWheelTickCount() {
  NO_ENCODER_IRQ {
    return right_wheel_num_ticks;
  }
  return 0;
}

WheelSpeedController::WheelSpeedController(
  WheelTickCountGetter * const wheel_tick_count_getter, 
  DutyCycleSetter * const duty_cycle_setter) 
  : wheel_tick_count_getter_(*ASSERT_NOT_NULL(wheel_tick_count_getter)), 
    duty_cycle_setter_(*ASSERT_NOT_NULL(duty_cycle_setter)), 
    last_run_seconds_(-1),
    pid_(kP, kI, kD)
{
}

float WheelSpeedController::DutyCycleFromLinearSpeed(float meters_per_second) const {
  // The wheel speed model was extracted experimentally here:
  // https://docs.google.com/spreadsheets/d/1u54eKSRv8ef7i6d9K4IdrqnENu4u1ELPQnM7t2nn1fk
  // 
  // wheel_speed = max(0, kSpeedModelSpeedOffset - kSpeedModelFactor * exp(-(duty_cycle + kSpeedModelDutyCycleOffset) / kSpeedModelTimeConstant))
  // 
  // This function computes the inverse of said model for both forward and backward speeds
  // and imposes duty cycle limits.
  if (meters_per_second >= kSpeedModelSpeedOffset) {
    return 1.0f;
  }
  if (meters_per_second <= -kSpeedModelSpeedOffset) {
    return -1.0f;
  }
  const float offset = kSpeedModelTimeConstant * log(kSpeedModelFactor) - kSpeedModelDutyCycleOffset;
  float duty_cycle = offset - kSpeedModelTimeConstant * log(kSpeedModelSpeedOffset - fabsf(meters_per_second));
  duty_cycle = min(1.0f, max(0.0f, duty_cycle));
  if (meters_per_second < 0) {
    duty_cycle = -duty_cycle;
  }
  return duty_cycle;
}

void WheelSpeedController::SetLinearSpeed(float meters_per_second) {  
  time_start_ = GetTimerSeconds();
  num_wheel_ticks_start_ = wheel_tick_count_getter_();
  pid_.target(meters_per_second);
}

void WheelSpeedController::SetAngularSpeed(float radians_per_second) {
  SetLinearSpeed(radians_per_second * kWheelRadius);
}

void WheelSpeedController::Run() {
  float now_seconds = GetTimerSeconds();
  if (last_run_seconds_ < 0) {
    last_run_seconds_ = now_seconds;
    return;
  }
  float seconds_since_last_run = now_seconds - last_run_seconds_;
  if (seconds_since_last_run <= kControlLoopPeriodSeconds) {
    return;
  }
  last_run_seconds_ = now_seconds;

  float seconds_since_start = now_seconds - time_start_;
  float average_wheel_speed = kWheelRadius * kRadiansPerWheelTick * (wheel_tick_count_getter_() - num_wheel_ticks_start_) / seconds_since_start;
  const float pid_output = pid_.update(average_wheel_speed, seconds_since_last_run);
  float speed_command = pid_.target() + pid_output;
  speed_command = max(speed_command, 0.0f);
  const float duty_cycle = DutyCycleFromLinearSpeed(speed_command);
  duty_cycle_setter_(duty_cycle);
  // Serial.printf("t:%f v:%f pid:%f c:%f d:%f\n", pid_.target(), average_wheel_speed, pid_output, speed_command, duty_cycle);
}

#include <limits>
#include "wheel_controller.h"
#include "robot_model.h"
#include "utils.h"
#include "encoders.h"

#define kSpeedModelTimeConstant 0.29
#define kSpeedModelDutyCycleOffset -0.99
#define kSpeedModelFactor 0.041
#define kSpeedModelSpeedOffset 0.66

#define kP 5.0
#define kI 0.0
#define kD 1.0

// Any wheel speed estimate above this value is rejected.
// Meant to prevent too high estimates due to co-occuring encoder edges.
#define kWheelRadiansPerSecondMax (2.0 / kWheelRadius) // [rad/s]

TimerTicksType left_wheel_last_ticks;
TimerTicksType right_wheel_last_ticks;

#define kMinSecondsPerWheelTick (kRadiansPerWheelTick / kWheelRadiansPerSecondMax)
#define kMinTimerTicksPerWheelTick (kMinSecondsPerWheelTick * kTimerTicksPerSecond)

float last_left_wheel_speed;
float last_right_wheel_speed;

static void LeftEncoderIsr(TimerTicksType timer_ticks) {
  last_left_wheel_speed = kRadiansPerWheelTick / SecondsFromTimerTicks(timer_ticks - left_wheel_last_ticks);
  left_wheel_last_ticks = timer_ticks;
}

static void RightEncoderIsr(TimerTicksType timer_ticks) {
  last_right_wheel_speed = kRadiansPerWheelTick / SecondsFromTimerTicks(timer_ticks - right_wheel_last_ticks);
  right_wheel_last_ticks = timer_ticks;
}

void InitWheelSpeedEstimator() {
  left_wheel_last_ticks = 0;
  right_wheel_last_ticks = 0;
  last_left_wheel_speed = 0;
  last_right_wheel_speed = 0;
  AddEncoderIsrs(&LeftEncoderIsr, &RightEncoderIsr);
}

float GetLeftWheelAngularSpeed() {
  NO_ENCODER_IRQ {
    return last_left_wheel_speed;
  }
}

float GetRightWheelAngularSpeed() {
  NO_ENCODER_IRQ {
    return last_right_wheel_speed;
  }
}

float GetLeftWheelLinearSpeed() {
  return GetLeftWheelAngularSpeed() * kWheelRadius;
}

float GetRightWheelLinearSpeed() {
  return GetRightWheelAngularSpeed() * kWheelRadius;
}

WheelSpeedController::WheelSpeedController(
  WheelSpeedGetter * const wheel_linear_speed_getter, 
  DutyCycleSetter * const duty_cycle_setter) 
  : wheel_linear_speed_getter_(*ASSERT_NOT_NULL(wheel_linear_speed_getter)), 
    duty_cycle_setter_(*ASSERT_NOT_NULL(duty_cycle_setter)), 
    pid_(kP, kI, kD)
{
}

float WheelSpeedController::DutyCycleFromLinearSpeed(float meters_per_second) const {
  // The wheel speed model was extracted experimentally here:
  // https://docs.google.com/spreadsheets/d/1u54eKSRv8ef7i6d9K4IdrqnENu4u1ELPQnM7t2nn1fk
  // 
  // wheel_speed = max(0, kSpeedModelSpeedOffset - kSpeedModelFactor * exp(-(duty_cycle + kSpeedModelDutyCycleOffset) / kSpeedModelTimeConstant))
  // 
  // This function computes the inverse of said model.
  const float offset = kSpeedModelTimeConstant * log(kSpeedModelFactor) - kSpeedModelDutyCycleOffset;
  return offset - kSpeedModelTimeConstant * log(kSpeedModelSpeedOffset - meters_per_second);
}

void WheelSpeedController::SetLinearSpeed(float meters_per_second) {  
  pid_.setTarget(meters_per_second);
}

void WheelSpeedController::SetAngularSpeed(float radians_per_second) {
  pid_.setTarget(radians_per_second * kWheelRadius);
}

void WheelSpeedController::Run() {
  pid_.setInput(wheel_linear_speed_getter_(), GetTimerNanoseconds() / 1000000);
  float speed_command = pid_.getTarget() + pid_.getOutput();
  speed_command = max(speed_command, 0.0f);
  float duty_cycle = DutyCycleFromLinearSpeed(speed_command);
  duty_cycle = max(min(duty_cycle, 1.0f), 0.0f);
  // if (speed_command < 0) {
  //   duty_cycle = -duty_cycle;
  // }
  duty_cycle_setter_(duty_cycle);    
  Serial.printf("t:%f v:%f pid:%f c:%f d:%f | ", pid_.getTarget(), wheel_linear_speed_getter_(), pid_.getOutput(), speed_command, duty_cycle);
}

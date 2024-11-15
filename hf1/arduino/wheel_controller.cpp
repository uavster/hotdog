#include <limits>
#include "wheel_controller.h"
#include "robot_model.h"
#include "utils.h"
#include "logger_interface.h"
#include <algorithm>

#define kControlLoopPeriodSeconds 1e-2

#define kSpeedModelTimeConstant 0.29
#define kSpeedModelDutyCycleOffset -0.99
#define kSpeedModelFactor 0.041
#define kSpeedModelSpeedOffset 0.66

// Only update the average speed estimate after this time period to ensure it is stable.
// Outer control loops must not change the target speed faster than this period, or the
// speed will never get re-estimated.
#define kSpeedMeasureUpdateStartSeconds 0.05

// Best params at 0.3 m/s forward.
#define kP 3.0
#define kI 4.0
#define kD 0.0

// This is meant to prevent large target speed steps from creating large wheel accelerations
// that could make the wheel slip (or tumble the robot in reverse).
// It takes kSpeedTargetRampExtraSecondsPerUnitSpeedIncrement * (next_target - current_target)^2 + kControlLoopPeriodSeconds
// seconds to reach the next_target speed from current_target speed. The higher the step in
// target speed the softer the ramp.
#define kSpeedTargetRampExtraSecondsPerUnitSpeedIncrement 4.0
#define kSpeedTargetRampExtraSecondsPerUnitSpeedIncrementReverse 8.0

#define kPWMDutyCycleMin 0.0f
#define kPWMDutyCycleMax 1.0f

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
  WheelTickCountGetter* const wheel_tick_count_getter,
  DutyCycleSetter* const duty_cycle_setter)
  : Controller(kControlLoopPeriodSeconds),
    wheel_tick_count_getter_(*ASSERT_NOT_NULL(wheel_tick_count_getter)),
    duty_cycle_setter_(*ASSERT_NOT_NULL(duty_cycle_setter)),
    last_run_seconds_(-1),
    average_wheel_speed_(0),
    is_turning_forward_(true),
    target_speed_(0),
    initial_target_speed_(0),
    target_speed_slope_(0),
    pid_(kP, kI, kD) {
}

float WheelSpeedController::DutyCycleFromLinearSpeed(float meters_per_second) const {
  // The wheel speed model was extracted experimentally here:
  // https://docs.google.com/spreadsheets/d/1u54eKSRv8ef7i6d9K4IdrqnENu4u1ELPQnM7t2nn1fk
  //
  // wheel_speed = max(0, kSpeedModelSpeedOffset - kSpeedModelFactor * exp(-(duty_cycle + kSpeedModelDutyCycleOffset) / kSpeedModelTimeConstant))
  //
  // This function computes the inverse of said model for both forward and backward speeds
  // and imposes duty cycle limits.
  if (meters_per_second == 0) {
    // If speed is 0, force the duty cycle to 0 to save power.
    return 0;
  }
  if (meters_per_second >= kSpeedModelSpeedOffset) {
    return 1.0f;
  }
  if (meters_per_second <= -kSpeedModelSpeedOffset) {
    return -1.0f;
  }
  const float offset = kSpeedModelTimeConstant * log(kSpeedModelFactor) - kSpeedModelDutyCycleOffset;
  float duty_cycle = offset - kSpeedModelTimeConstant * log(kSpeedModelSpeedOffset - fabsf(meters_per_second));
  duty_cycle = std::clamp(duty_cycle, kPWMDutyCycleMin, kPWMDutyCycleMax);
  if (meters_per_second < 0) {
    duty_cycle = -duty_cycle;
  }
  return duty_cycle;
}

void WheelSpeedController::SetLinearSpeed(float meters_per_second) {
  time_start_ = GetTimerSeconds();
  num_wheel_ticks_start_ = wheel_tick_count_getter_();
  target_speed_ = meters_per_second;
  initial_target_speed_ = pid_.target();
  const float ramp_extra_seconds = meters_per_second >= 0 ? kSpeedTargetRampExtraSecondsPerUnitSpeedIncrement : kSpeedTargetRampExtraSecondsPerUnitSpeedIncrementReverse;
  target_speed_slope_ = 1 / (kControlLoopPeriodSeconds + ramp_extra_seconds * abs(meters_per_second - initial_target_speed_));
  if (meters_per_second < 0) {
    target_speed_slope_ = -target_speed_slope_;
  }
}

void WheelSpeedController::SetAngularSpeed(float radians_per_second) {
  SetLinearSpeed(radians_per_second * kWheelRadius);
}

void WheelSpeedController::Update(TimerSecondsType now_seconds) {
  const float seconds_since_start = now_seconds - time_start_;  

  // Ramp up or down the speed target.
  const float current_target = initial_target_speed_ + seconds_since_start * target_speed_slope_;
  pid_.target(target_speed_ >= 0 ? min(target_speed_, current_target) : max(target_speed_, current_target));
  
  // Estimate wheel turn direction.
  // Assume the wheel is turning in the commanded direction because we cannot sense it.
  // If the target is zero, it does not work as proxy of the speed sign, so infer the
  // sign from the previous pid output. Without this, when the target is zero, a speed
  // error can destabilize the control loop and make the robot drive backwards
  // indefinitely.
  bool is_turning_forward = pid_.target() > 0 || (pid_.target() == 0 && pid_.output() >= 0);
  if (is_turning_forward != is_turning_forward_) {
    // The turn direction changed: reset the speed estimation.
    average_wheel_speed_ = 0;
  }
  is_turning_forward_ = is_turning_forward;

  // Estimate wheel speed.
  int num_encoder_ticks = wheel_tick_count_getter_() - num_wheel_ticks_start_;
  if (num_encoder_ticks > 0) {
    // Only update the speed estimate if got any encoder ticks since the last change of
    // target speed. Otherwise, we'd get 0 after every change and the consequent control
    // peak.
    if (seconds_since_start > kSpeedMeasureUpdateStartSeconds) {
      // Only update the speed estimate after some time to ensure that it is stable.
      // Otherwise, we get artificial peaks in the control action.
      float cur_avg_speed = kWheelRadius * kRadiansPerWheelTick * num_encoder_ticks / seconds_since_start;
      if (!is_turning_forward_) {
        // Assume the wheel is turning in the commanded direction because we cannot sense it.
        cur_avg_speed = -cur_avg_speed;
      }
      average_wheel_speed_ = cur_avg_speed;
    }
  }

  // Update duty cycle with the speed estimate.
  const float pid_output = pid_.update(average_wheel_speed_);
  float speed_command = pid_.target() + pid_output;
  if ((is_turning_forward && speed_command < 0) || (!is_turning_forward && speed_command > 0)) {
    // Avoid speed commands opposite to the driving direction as that can make the wheel
    // slip and hurt localization, making the error irrecoverable by the trajectory
    // controller.
    speed_command = 0;
  }
  const float duty_cycle = DutyCycleFromLinearSpeed(speed_command);
  duty_cycle_setter_(duty_cycle);

  // Serial.printf("t:%f v:%f pid:%f c:%f d:%f\n", pid_.target(), average_wheel_speed_, pid_output, speed_command, duty_cycle);
}

float WheelSpeedController::GetMaxLinearSpeed() const {
  const float offset = kSpeedModelTimeConstant * log(kSpeedModelFactor) - kSpeedModelDutyCycleOffset;
  return kSpeedModelSpeedOffset - exp((offset - kPWMDutyCycleMax) / kSpeedModelTimeConstant);
}

float WheelSpeedController::GetMaxAngularSpeed() const {
  return GetMaxLinearSpeed() / kWheelRadius;
}

float WheelSpeedController::GetMinLinearSpeed() const {
  return -GetMaxLinearSpeed();
}

float WheelSpeedController::GetMinAngularSpeed() const {
  return -GetMaxAngularSpeed();
}

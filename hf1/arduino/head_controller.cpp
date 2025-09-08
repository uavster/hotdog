#include <new>
#include "core_pins.h"
#include "head_controller.h"
#include "servos.h"
#include <Arduino.h>

constexpr float kHeadPositionControllerUpdatePeriodSeconds = 0.01;

// Time between calls to the head trajectory controller.
// Times shorter than the servo command update rate have no effect.
#define kHeadTrajectoryControllerLoopPeriodSeconds 0.01

HeadPositionController::HeadPositionController(const char *name) : Controller(name, kHeadPositionControllerUpdatePeriodSeconds) {
  SetTarget(0, 0, 0);
}

void HeadPositionController::Update(TimerSecondsType seconds_since_start) {
  const float measured_pitch_degrees = -(((analogRead(A1) - pitch_feedback_limits_.min_adc_reading) * (GetPitchServoMaxDegrees() - GetPitchServoMinDegrees())) / (pitch_feedback_limits_.max_adc_reading - pitch_feedback_limits_.min_adc_reading) + GetPitchServoMinDegrees());
  Serial.printf("%f: %f (%f)\n", seconds_since_start, measured_pitch_degrees, RadiansFromDegrees(measured_pitch_degrees));

  const float pitch_error_degrees = pitch_target_degrees_ - measured_pitch_degrees;

  switch(pitch_servo_state_) {
    case kNewTarget: {  // The error deviated too much from 0: measure its change.
      if (fabsf(pitch_error_degrees - prev_pitch_error_degrees_) < 0.5) {
        pitch_servo_state_ = kReachedTarget;
        break;
      }
      const float error_change = fabsf(pitch_error_degrees) - fabsf(prev_pitch_error_degrees_);
      // Both errors are at the same side of the target.
      if (error_change > 0) {
        pitch_servo_state_ = kChangingDirection;
        break;
      }
      if (error_change < 0) {
        pitch_servo_state_ = kApproximatingTarget;
        break;
      }
      break;
    }

    case kChangingDirection: {  // The position is increasing after setting a new target because the servo was moving in the opposite direction.
      if (pitch_error_degrees * prev_pitch_error_degrees_ < 0) {
        pitch_servo_state_ = kOvershootingTarget;
        break;
      }
      // Both errors are at the same side of the target.
      const float error_change = fabsf(pitch_error_degrees) - fabsf(prev_pitch_error_degrees_);
      if (error_change < 0) {
        pitch_servo_state_ = kApproximatingTarget;
        break;
      }
  
      break;
    }

    case kApproximatingTarget: {  // The position error is decreasing.
      if (fabsf(pitch_error_degrees - prev_pitch_error_degrees_) < 0.5) {
        pitch_servo_state_ = kReachedTarget;
        break;
      }
      if (pitch_error_degrees * prev_pitch_error_degrees_ < 0) {
        pitch_servo_state_ = kOvershootingTarget;
        break;
      }
      const float error_change = pitch_error_degrees - prev_pitch_error_degrees_;
      if (error_change > 0) {
        pitch_servo_state_ = kChangingDirection;
        break;
      }
      break;
    }

    case kOvershootingTarget: {  // The position error changed sign.
      if (fabsf(pitch_error_degrees - prev_pitch_error_degrees_) < 0.5) {
        pitch_servo_state_ = kReachedTarget;
        break;
      }
      break;
    }

    case kReachedTarget: { // The target was reached.
      if (fabsf(pitch_error_degrees - prev_pitch_error_degrees_) > 0.5) {
        pitch_servo_state_ = kNewTarget;
        break;
      }
      break;
    }
  }
  Serial.printf("e:%f prev_e:%f new_state:%d\n", pitch_error_degrees, prev_pitch_error_degrees_, pitch_servo_state_);

  prev_pitch_error_degrees_ = pitch_error_degrees;
}

HeadTrajectoryController::HeadTrajectoryController(const char *name) 
  : TrajectoryController<HeadTargetState>(name, kHeadTrajectoryControllerLoopPeriodSeconds) {}

void HeadTrajectoryController::Update(TimerSecondsType seconds_since_start) {
  TrajectoryController<HeadTargetState>::Update(seconds_since_start);
  if (!is_started()) { return; }
  
  const State ref_position = trajectory().state(seconds_since_start);
  SetHeadYawDegrees(DegreesFromRadians(ref_position.location().yaw()));
  SetHeadPitchDegrees(DegreesFromRadians(ref_position.location().pitch()));
  SetHeadRollDegrees(DegreesFromRadians(ref_position.location().roll()));
}
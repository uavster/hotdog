#ifndef HEAD_CONTROLLER_
#define HEAD_CONTROLLER_

#include "controller.h"
#include "head_state.h"
#include "servos.h"

// Ensures the servos reach the target position without jerking.
class HeadPositionController : public Controller {
public:
  HeadPositionController(const char *name);

  struct JointFeedbackLimits {
    uint16_t min_adc_reading;
    uint16_t max_adc_reading;
  };

  void SetJointFeedbackLimits(const JointFeedbackLimits &yaw, const JointFeedbackLimits &pitch, const JointFeedbackLimits &roll) {
    yaw_feedback_limits_ = yaw;
    pitch_feedback_limits_ = pitch;
    roll_feedback_limits_ = roll;
  }

  void SetTarget(float yaw_radians, float pitch_radians, float roll_radians) {
    yaw_target_degrees_ = DegreesFromRadians(yaw_radians);
    pitch_target_degrees_ = DegreesFromRadians(pitch_radians);
    roll_target_degrees_ = DegreesFromRadians(roll_radians);
    SetHeadYawDegrees(yaw_target_degrees_);
    SetHeadPitchDegrees(pitch_target_degrees_);
    SetHeadRollDegrees(roll_target_degrees_);
    pitch_servo_state_ = kNewTarget;
  }

protected:
  void Update(TimerSecondsType seconds_since_start) override;

private:
  JointFeedbackLimits yaw_feedback_limits_;
  JointFeedbackLimits pitch_feedback_limits_;
  JointFeedbackLimits roll_feedback_limits_;

  float yaw_target_degrees_;
  float pitch_target_degrees_;
  float roll_target_degrees_;

  enum State {
    kNewTarget,  // The error deviated too much from 0: measure its change.
    kChangingDirection,  // The position is increasing after setting a new target because the servo was moving in the opposite direction.
    kApproximatingTarget, // The position error is decreasing.
    kOvershootingTarget,  // The position error changed sign.
    kReachedTarget, // The target was reached.
  };
  State pitch_servo_state_;

  float prev_pitch_error_degrees_;
};

class HeadTrajectoryController : public TrajectoryController<HeadTargetState> {
public:
  HeadTrajectoryController(const char *name);

protected:
  virtual void Update(TimerSecondsType seconds_since_start) override;
};

#endif  // HEAD_CONTROLLER_
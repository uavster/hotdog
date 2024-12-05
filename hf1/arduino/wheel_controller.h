#ifndef WHEEL_CONTROLLER_INCLUDED_
#define WHEEL_CONTROLLER_INCLUDED_

#include "encoders.h"
#include "pid.h"
#include "controller.h"
#include "robot_model.h"
#include "wheel_state_estimator.h"

typedef void (DutyCycleSetter)(float duty_cycle);

class WheelSpeedController : public Controller {
public:
  // No ownership of the pointee is taken by this object.
  WheelSpeedController(const char *name, const WheelStateFilter * const wheel_state_filter_, DutyCycleSetter * const duty_cycle_setter);

  // Returns the maximum attainable linear speed [m/s].
  float GetMaxLinearSpeed() const;
  // Returns the maximum attainable angular speed [rad/s].
  float GetMaxAngularSpeed() const;
  // Returns the minimum (most negative) attainable linear speed [m/s].
  float GetMinLinearSpeed() const;
  // Returns the minimum (most negative) attainable angular speed [rad/s].
  float GetMinAngularSpeed() const;

  // Sets the target speed of the wheel in m/s (linear) or rad/s (angular). Negative values 
  // make the wheel move backwards. The caller must ensure the current speed is zero before 
  // setting a target speed opposite to the current direction of movement. Otherwise, the 
  // sudden change would introduce localization error, and the robot might even tumble.
  void SetLinearSpeed(float meters_per_second);
  void SetAngularSpeed(float radians_per_second);

  float GetLinearSpeed() const { return pid_.target(); }
  float GetAngularSpeed() const { return GetLinearSpeed() / kWheelRadius; }

  bool is_turning_forward() const { return is_turning_forward_; }

protected:
  // Periodically updates the speed controller.
  void Update(TimerSecondsType now_seconds) override;

private:
  float DutyCycleFromLinearSpeed(float meters_per_second) const;

  const WheelStateFilter &wheel_state_filter_;
  DutyCycleSetter &duty_cycle_setter_;

  float last_run_seconds_;
  float time_start_;
  bool is_turning_forward_;
  float target_speed_;
  float initial_target_speed_;
  float target_speed_slope_;
  
  PID pid_;
};

#endif  // WHEEL_CONTROLLER_INCLUDED_

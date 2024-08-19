#ifndef WHEEL_CONTROLLER_INCLUDED_
#define WHEEL_CONTROLLER_INCLUDED_

#include <PID.h>

void InitWheelSpeedEstimator();

float GetLeftWheelAngularSpeed();
float GetRightWheelAngularSpeed();

float GetLeftWheelLinearSpeed();
float GetRightWheelLinearSpeed();

typedef void (DutyCycleSetter)(float duty_cycle);
typedef float (WheelSpeedGetter)(void);

class WheelSpeedController {
public:
  // No ownership of the pointee is taken by this object.
  WheelSpeedController(WheelSpeedGetter * const wheel_linear_speed_getter, DutyCycleSetter * const duty_cycle_setter);

  void SetLinearSpeed(float meters_per_second);
  void SetAngularSpeed(float radians_per_second);

  void Run();

private:
  float DutyCycleFromLinearSpeed(float meters_per_second) const;

  WheelSpeedGetter &wheel_linear_speed_getter_;
  DutyCycleSetter &duty_cycle_setter_;
  arc::PID<float> pid_;
};

#endif  // WHEEL_CONTROLLER_INCLUDED_

#ifndef WHEEL_CONTROLLER_INCLUDED_
#define WHEEL_CONTROLLER_INCLUDED_

#include "encoders.h"
#include "pid.h"

void InitWheelSpeedControl();

int32_t GetLeftWheelTickCount();
int32_t GetRightWheelTickCount();

typedef void (DutyCycleSetter)(float duty_cycle);
typedef int32_t (WheelTickCountGetter)(void);

class WheelSpeedController {
public:
  // No ownership of the pointee is taken by this object.
  WheelSpeedController(WheelTickCountGetter * const wheel_tick_count_getter, DutyCycleSetter * const duty_cycle_setter);

  void SetLinearSpeed(float meters_per_second);
  void SetAngularSpeed(float radians_per_second);

  void Run();

private:
  float DutyCycleFromLinearSpeed(float meters_per_second) const;

  WheelTickCountGetter &wheel_tick_count_getter_;
  DutyCycleSetter &duty_cycle_setter_;

  float last_run_seconds_;
  float time_start_;
  int32_t num_wheel_ticks_start_;  
  
  PID pid_;
};

#endif  // WHEEL_CONTROLLER_INCLUDED_

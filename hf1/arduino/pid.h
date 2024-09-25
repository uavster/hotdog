#ifndef PID_INCLUDED_
#define PID_INCLUDED_

#include "timer.h"

class PID {
public:
  PID(float p, float i, float d);

  // Sets the target value.
  void target(float target);
  // Returns the target value.
  float target() const { return target_; }

  // Updates the PID output and returns it.
  float update(float feedback);

  // Returns the PID output.
  float output() const { return output_; }

  float p() const { return kp_; }
  void p(float p) { kp_ = p; }

  float i() const { return ki_; }
  void i(float i) { ki_ = i; }

  float d() const { return kd_; }
  void d(float d) { kd_ = d; }

  void setParameters(float p, float i, float d) { kp_ = p; ki_ = i; kd_ = d; }

private:
  float kp_;
  float ki_;
  float kd_;
  float target_;
  float integrator_;
  float last_error_;
  float output_;
  bool after_reset_;
  TimerSecondsType last_update_seconds_;
};

#endif  // PID_INCLUDED_
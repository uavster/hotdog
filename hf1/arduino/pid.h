#ifndef PID_INCLUDED_
#define PID_INCLUDED_

class PID {
public:
  PID(float p, float i, float d);

  void target(float target) { target_ = target; }
  float target() const { return target_; }
  void ResetIntegrator() { integrator_ = 0; }

  float update(float feedback, float seconds_since_last_update);

private:
  float kp_;
  float ki_;
  float kd_;
  float target_;
  float integrator_;
  float error_;
};

#endif  // PID_INCLUDED_
#ifndef PID_INCLUDED_
#define PID_INCLUDED_

class PID {
public:
  PID(float p, float i, float d);

  // Sets the target value.
  void target(float target) { target_ = target; }
  // Returns the target value.
  float target() const { return target_; }

  // Sets the integrator term to zero. When the target is updated to a different value, 
  // the integrator will keep pushing the output towards the old target, so it is better
  // to reset it.
  void ResetIntegrator() { integrator_ = 0; }

  // Updates the PID output and returns it.
  float update(float feedback, float seconds_since_last_update);

  // Returns the PID output.
  float output() const { return output_; }

private:
  float kp_;
  float ki_;
  float kd_;
  float target_;
  float integrator_;
  float error_;
  float output_;
};

#endif  // PID_INCLUDED_
#include "pid.h"

PID::PID(float kp, float ki, float kd) 
  : kp_(kp), ki_(ki), kd_(kd), target_(0), integrator_(0), last_error_(0), output_(0), after_reset_(true) {}

void PID::target(float target) {
  after_reset_ = true;
  target_ = target;
}

float PID::update(float feedback) {
  const float now_seconds = GetTimerSeconds();
  const float error = target_ - feedback;
  if (after_reset_) {
    // Sets the integrator term to zero. Otherwise, the integrator would keep pushing the 
    // output towards the old target after the target is updated.
    integrator_ = 0;
    // Reset calculation of integrative and derivative components.
    last_update_seconds_ = now_seconds;
    after_reset_ = false;
  }

  if (error * last_error_ <= 0) {
    // When error crosses 0, reset the integrator to prevent wind-up.
    integrator_ = 0;
  }
  const float seconds_since_last_update = now_seconds - last_update_seconds_;
  last_update_seconds_ = now_seconds;
  integrator_ += error * seconds_since_last_update;
  float derivative = 0;
  if (seconds_since_last_update > 0) {
    derivative = (error - last_error_) / seconds_since_last_update;
  }
  last_error_ = error;
  output_ = kp_ * error + ki_ * integrator_ + kd_ * derivative;
  return output_;
}

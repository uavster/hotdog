#include "pid.h"

PID::PID(float kp, float ki, float kd) 
  : kp_(kp), ki_(ki), kd_(kd), target_(0), integrator_(0), error_(0) {}

float PID::update(float feedback, float seconds_since_last_update) {
  float error = target_ - feedback;
  integrator_ += ki_ * error * seconds_since_last_update;
  if (error * error_ <= 0) {
    // When error crosses 0, reset the integrator to avoid wind-up.
    integrator_ = 0;
  }
  float derivative = (error - error_) / seconds_since_last_update;
  error_ = error;
  return kp_ * error + ki_ * integrator_ + kd_ * derivative;
}

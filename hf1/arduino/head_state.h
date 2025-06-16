#ifndef HEAD_STATE_INCLUDED_
#define HEAD_STATE_INCLUDED_

#include "state.h"
#include "vector.h"
#include <math.h>

class HeadStateVars {
public:
  HeadStateVars() : yaw_(0), pitch_(0), roll_(0) {}
  HeadStateVars(float yaw, float pitch, float roll) : yaw_(yaw), pitch_(pitch), roll_(roll) {}

  float yaw() const { return yaw_; }
  float pitch() const { return pitch_; }
  float roll() const { return roll_; }

  HeadStateVars operator+(const HeadStateVars &other) const {
    return HeadStateVars(yaw_ + other.yaw_, pitch_ + other.pitch_, roll_ + other.roll_);
  }

  HeadStateVars operator*(float factor) const {
    return HeadStateVars(yaw_ * factor, pitch_ * factor, roll_ * factor);
  }

  float DistanceFrom(const HeadStateVars &state) const {
    const float yaw_diff = yaw() - state.yaw();
    const float pitch_diff = pitch() - state.pitch();
    const float pitch_roll = roll() - state.roll();
    return sqrtf(yaw_diff * yaw_diff + pitch_diff * pitch_diff + pitch_roll * pitch_roll);
  }

private:
  float yaw_;
  float pitch_;
  float roll_;
};

// Target state of the head.
using HeadTargetState = State<HeadStateVars, /*Order=*/0>;

#endif  // HEAD_STATE_INCLUDED_
#ifndef HEAD_STATE_INCLUDED_
#define HEAD_STATE_INCLUDED_

#include "state.h"
#include "point.h"

class HeadStateVars {
public:
  HeadStateVars() : pitch_(0), roll_(0) {}
  HeadStateVars(float pitch, float roll) : pitch_(pitch), roll_(roll) {}

  float pitch() const { return pitch_; }
  float roll() const { return roll_; }

  HeadStateVars operator+(const HeadStateVars &other) const {
    return HeadStateVars(pitch_ + other.pitch_, roll_ + other.roll_);
  }

  HeadStateVars operator*(float factor) const {
    return HeadStateVars(pitch_ * factor, roll_ * factor);
  }

private:
  float pitch_;
  float roll_;
};

using HeadState = State<HeadStateVars, 1>;

#endif  // HEAD_STATE_INCLUDED_
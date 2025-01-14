#ifndef BASE_STATE_INCLUDED_
#define BASE_STATE_INCLUDED_

#include "state.h"
#include "point.h"

class BaseStateVars {
public:
  BaseStateVars() : yaw_(0) {}
  BaseStateVars(const Point &position, float yaw) : position_(position), yaw_(yaw) {}

  const Point &position() const { return position_; }
  float yaw() const { return yaw_; }

  BaseStateVars operator+(const BaseStateVars &other) const {
    return BaseStateVars(position_ + other.position_, yaw_ + other.yaw_);
  }

  BaseStateVars operator*(float factor) const {
    return BaseStateVars(position_ * factor, yaw_ * factor);
  }

  float DistanceFrom(const BaseStateVars &state) const {
    return (position() - state.position()).norm();
  }

private:
  Point position_;
  float yaw_;
};

// Estimated state of the base.
using BaseState = State<BaseStateVars, 1>;

// Target state of the base.
using BaseTargetState = State<BaseStateVars, 0>;

#endif  // BASE_STATE_INCLUDED_
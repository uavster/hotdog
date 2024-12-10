#ifndef TRAJECTORY_INCLUDED_
#define TRAJECTORY_INCLUDED_

#include "status_or.h"
#include "timer.h"

// Defines a state at a given time.
template<typename TState>
class Waypoint {
public:
  Waypoint() : seconds_(0) {}
  Waypoint(TimerSecondsType seconds, const TState &state) 
    : seconds_(seconds), state_(state) {}

  TimerSecondsType seconds() const { return seconds_; }
  const TState &state() const { return state_; }

  Waypoint operator+(const Waypoint &other) const {
    return Waypoint(seconds_ + other.seconds_, state_ + other.state_);
  }

  Waypoint operator-(const Waypoint &other) const {
    return Waypoint(seconds_ - other.seconds_, state_ - other.state_);
  }

  Waypoint operator*(float factor) const {
    return Waypoint(seconds_ * factor, state_ * factor);
  }

  Waypoint operator/(float divisor) const {
    return Waypoint(seconds_ / divisor, state_ / divisor);
  }

private:
  TimerSecondsType seconds_;
  TState state_;
};

template<typename TState>
class TrajectoryInterface {
public:
  virtual int size() const = 0;  
  virtual const Waypoint<TState> &operator[](int i) const = 0;
};

// A collection of waypoints sorted by time.
template<typename TState, int Capacity>
class Trajectory : public TrajectoryInterface<TState> {
  static_assert(Capacity > 0);
public:
  Trajectory() : size_(0) {}
  template<int Size> Trajectory(const Waypoint<TState> (&waypoints)[Size]);
  Trajectory(int num_waypoints, const Waypoint<TState> *waypoints);

  int capacity() const { return Capacity; }
  int size() const override { return size_; }
    
  const Waypoint<TState> &operator[](int i) const override;

  void Insert(const Waypoint<TState> &waypoint);

private:
  int FindInsertionIndex(float seconds, int start_index, int end_index) const;

  int size_;
  Waypoint<TState> waypoints_[Capacity];
};

#include "trajectory.hh"

#endif  // TRAJECTORY_INCLUDED_
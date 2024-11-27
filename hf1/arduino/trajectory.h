#ifndef TRAJECTORY_INCLUDED_
#define TRAJECTORY_INCLUDED_

#include "status_or.h"

// Defines a state of the robot at a given time. 
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

// Cubic interpolation uses centripetal Catmull-Rom splines.
typedef enum { kNone, kLinear, kCubic } InterpolationType;
typedef struct {
  InterpolationType type;
  int num_sampling_points;
} InterpolationConfig;

// A view to a collection of waypoints.
// The view does not own the memory containing the waypoints, so they must outlive any 
// view objects referencing them.
template<typename TState>
class TrajectoryView {
public:
  TrajectoryView() : num_waypoints_(0), waypoints_(NULL), loop_after_seconds_(-1) {}

  // Does not take ownsership of the pointee, which must outlive this object.
  TrajectoryView(int num_waypoints, const Waypoint<TState> *waypoints);

  // Returns true if the trajectory is valid, e.g. no two waypoints defined for the same time.
  static bool IsTrajectoryValid(int num_waypoints, const Waypoint<TState> *waypoints);

  // Returns the number of waypoints in the trajectory, after applying interpolation.
  // If interpolation is enabled, this won't match the number of waypoints passed to the
  // constructor. The valid range is then [0, num_sampling_points).
  int NumWaypoints() const;

  // Returns the waypoint at the given index, after applying interpolation.
  // If interpolation is enabled, the valid index range is [0, num_sampling_points).
  // Otherwise, it is [0, num_waypoints];
  Waypoint<TState> GetWaypoint(int index) const;

  // Returns the duration of one trajectory lap. 
  // If no looping is enabled, this is the time between the first and last waypoints.
  // If looping is enabled, this is the time above plus the time it takes to return to the 
  // starting waypoint.
  float LapDuration() const;

  // `after_seconds` must be enough time for the controller to take the state from the last
  // waypoint to the first one.
  TrajectoryView &EnableLooping(TimerSecondsType after_seconds);
  TrajectoryView &DisableLooping();
  bool IsLoopingEnabled() const;
  StatusOr<TimerSecondsType> SecondsBetweenLoops() const;

  TrajectoryView &EnableInterpolation(const InterpolationConfig &config);
  TrajectoryView &DisableInterpolation();

  StatusOr<int> FindWaypointIndexBeforeSeconds(TimerSecondsType seconds, int prev_result_index = 0) const;

  TState state(int index) const;
  TState derivative(int order, int index) const;
  float seconds(int index) const;

private:
  // Returns the waypoint without interpolation for the given index, assuming a periodic
  // trajectory.
  Waypoint<TState> GetPeriodicWaypoint(int index) const;

  int num_waypoints_;
  const Waypoint<TState> *waypoints_;
  InterpolationConfig interpolation_config_;
  TimerSecondsType loop_after_seconds_; // looping disabled if negative.  
};

#include "trajectory.hh"

#endif  // TRAJECTORY_INCLUDED_
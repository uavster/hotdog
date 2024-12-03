#ifndef TRAJECTORY_INCLUDED_
#define TRAJECTORY_INCLUDED_

#include "status_or.h"

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

// Cubic interpolation uses centripetal Catmull-Rom splines.
typedef enum { kNone, kLinear, kCubic } InterpolationType;
typedef struct {
  InterpolationType type;
  TimerSecondsType sampling_period_seconds;
} InterpolationConfig;

// Base class of trajectories passed to descendants of TrajectoryController.
template<typename TState>
class TrajectoryViewInterface {
public:
  // Returns the number of waypoints in the trajectory, after applying interpolation.
  virtual int NumWaypoints() const = 0;

  // Returns the waypoint at the given index, after applying interpolation.
  virtual Waypoint<TState> GetWaypoint(int index) const = 0;

  // Returns true if the trajectory is set to restart from the beginning some time after
  // reaching the end.
  virtual bool IsLoopingEnabled() const = 0;

  float seconds(int index) const;
  TState state(int index) const;
  TState derivative(int order, int index) const;
  int FindWaypointIndexBeforeSeconds(TimerSecondsType seconds, int prev_result_index = 0) const;
};

// A view to a collection of waypoints.
// The view does not own the memory containing the waypoints, so they must outlive any 
// view objects referencing them.
template<typename TState>
class TrajectoryView : public TrajectoryViewInterface<TState> {
public:
  TrajectoryView() : num_waypoints_(0), waypoints_(NULL), loop_after_seconds_(-1) {}

  // Does not take ownsership of the pointee, which must outlive this object.
  TrajectoryView(int num_waypoints, const Waypoint<TState> *waypoints);

  // Returns true if the trajectory is valid, e.g. no two waypoints defined for the same time.
  static bool IsTrajectoryValid(int num_waypoints, const Waypoint<TState> *waypoints);

  // Returns the number of waypoints in the trajectory, after applying interpolation.
  // If interpolation is enabled, this won't match the number of waypoints passed to the
  // constructor. The valid range is then [0, num_sampling_points).
  int NumWaypoints() const override;

  // Returns the waypoint at the given index, after applying interpolation.
  // If interpolation is enabled, the valid index range is [0, num_sampling_points).
  // Otherwise, it is [0, num_waypoints];
  Waypoint<TState> GetWaypoint(int index) const override;

  // Returns the duration of one trajectory lap. 
  // If no looping is enabled, this is the time between the first and last waypoints.
  // If looping is enabled, this is the time above plus the time it takes to return to the 
  // starting waypoint.
  float LapDuration() const;

  // `after_seconds` must be enough time for the controller to take the state from the last
  // waypoint to the first one.
  TrajectoryView &EnableLooping(TimerSecondsType after_seconds);
  TrajectoryView &DisableLooping();
  bool IsLoopingEnabled() const override;
  StatusOr<TimerSecondsType> SecondsBetweenLoops() const;

  TrajectoryView &EnableInterpolation(const InterpolationConfig &config);
  TrajectoryView &DisableInterpolation();
  const InterpolationConfig &interpolation_config() const { return interpolation_config_; }

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
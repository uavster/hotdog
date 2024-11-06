#ifndef TRAJECTORY_INCLUDED_
#define TRAJECTORY_INCLUDED_

#include "status_or.h"

// Defines a state of the robot at a given time. 
template<typename TState> class Waypoint {
public:
  Waypoint() : seconds_(0) {}
  Waypoint(TimerSecondsType seconds, const TState &state) 
    : seconds_(seconds), state_(state) {}

  TimerSecondsType seconds() const { return seconds_; }
  const TState &state() const { return state_; }

private:
  TimerSecondsType seconds_;
  TState state_;
};

// A view to a collection of waypoints.
// The view does not own the memory containing the waypoints, so they must outlive any 
// view objects referencing them.
template<typename TState> class TrajectoryView {
public:
  TrajectoryView() : num_waypoints_(0), waypoints_(NULL), loop_at_seconds_(-1) {}

  // Does not take ownsership of the pointee, which must outlive this object.
  TrajectoryView(int num_waypoints, const Waypoint<TState> *waypoints)
    : num_waypoints_(num_waypoints), waypoints_(waypoints), loop_at_seconds_(-1) {}

  int num_waypoints() const { return num_waypoints_; }

  TrajectoryView &EnableLooping(TimerSecondsType after_seconds = 0);
  TrajectoryView &DisableLooping();
  bool IsLoopingEnabled() const;

  StatusOr<int> FindWaypointIndexBeforeSeconds(TimerSecondsType seconds, int prev_result_index = 0) const;

  const TState &state(int index) const;
  TState derivative(int order, int index) const;
  float seconds(int index) const;

private:
  int num_waypoints_;
  const Waypoint<TState> *waypoints_;
  TimerSecondsType loop_at_seconds_; // looping disabled if negative.
};

#include "trajectory.hh"

#endif  // TRAJECTORY_INCLUDED_
#ifndef TRAJECTORY_VIEW_INCLUDED_
#define TRAJECTORY_VIEW_INCLUDED_

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

// A view to a trajectory.
// It can upsample the trajectory with interpolation on the fly, and loop the
// trajectory after a given interval.
// The view does not own the trajectory, which must outlive any view objects 
// referencing it.
template<typename TState>
class TrajectoryView : public TrajectoryViewInterface<TState> {
public:
  TrajectoryView() : trajectory_(nullptr) {}
  // Does not take ownsership of the pointee, which must outlive this object.
  TrajectoryView(const TrajectoryInterface<TState> *trajectory);

  // Returns the number of waypoints in the trajectory, after applying interpolation.
  // If interpolation is enabled, this won't match the number of waypoints passed to the
  // constructor. The valid range is then [0, num_sampling_points), where 
  // num_sampling_points = LapDuration() / sampling_period_seconds.
  int NumWaypoints() const override;

  // Returns the waypoint at the given index, after applying interpolation.
  // If interpolation is enabled, the valid index range is [0, num_sampling_points), where 
  // num_sampling_points = LapDuration() / sampling_period_seconds.
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

  // Returns the index of the waypoint immediately before `seconds` without interpolation.
  // int FindNonInterpolatedWaypointIndexBeforeSeconds(TimerSecondsType seconds) const;

  const TrajectoryInterface<TState> *trajectory_;
  InterpolationConfig interpolation_config_;
  TimerSecondsType loop_after_seconds_; // looping disabled if negative.  
};

#include "trajectory_view.hh"

#endif  // TRAJECTORY_VIEW_INCLUDED_
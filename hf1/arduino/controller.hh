#include "trajectory.h"

template<typename TrajectoryViewType>
TrajectoryController<TrajectoryViewType>::TrajectoryController(const char *name, const float run_period_seconds)
  : Controller(name, run_period_seconds),
    target_waypoint_index_(0),
    state_(kStopped) {}

template<typename TrajectoryViewType>
void TrajectoryController<TrajectoryViewType>::trajectory(const TrajectoryViewType &trajectory) {
  trajectory_ = trajectory;
  target_waypoint_index_ = 0;
  state_ = kStopped;
}

template<typename TrajectoryViewType>
void TrajectoryController<TrajectoryViewType>::StartTrajectory() {
  target_waypoint_index_ = 0;
  start_seconds_ = GetTimerSeconds();
  state_ = kFollowingTrajectory;
}

template<typename TrajectoryViewType>
void TrajectoryController<TrajectoryViewType>::StopTrajectory() {
  Stop();
  state_ = kStopped;
}

template<typename TrajectoryViewType>
void TrajectoryController<TrajectoryViewType>::Update(const TimerSecondsType now_seconds) {
  switch(state_) {
    case kStopped:
      break;
    case kFollowingTrajectory: {
      const TimerSecondsType seconds_since_start = now_seconds - start_seconds_;
      target_waypoint_index_ = trajectory_.FindWaypointIndexBeforeSeconds(seconds_since_start, target_waypoint_index_);

      Update(seconds_since_start, target_waypoint_index_);

      if (!trajectory_.IsLoopingEnabled() && 
          target_waypoint_index_ >= trajectory_.NumWaypoints() - 1) {
        StopTrajectory();
      }

      break;
    }
  }
}
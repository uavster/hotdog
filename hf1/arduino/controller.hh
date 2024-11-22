template<typename TrajectoryViewType>
TrajectoryController<TrajectoryViewType>::TrajectoryController(const float run_period_seconds)
  : Controller(run_period_seconds),
    state_(kStopped),
    current_waypoint_index_(0) {}

template<typename TrajectoryViewType>
void TrajectoryController<TrajectoryViewType>::trajectory(const TrajectoryViewType &trajectory) {
  trajectory_ = trajectory;
  current_waypoint_index_ = 0;
  state_ = kStopped;
}

template<typename TrajectoryViewType>
void TrajectoryController<TrajectoryViewType>::StartTrajectory() {
  current_waypoint_index_ = 0;
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
      const auto maybe_index = trajectory_.FindWaypointIndexBeforeSeconds(seconds_since_start, current_waypoint_index_);
      if (!maybe_index.ok()) { break; }
      current_waypoint_index_ = *maybe_index;
      Update(seconds_since_start, current_waypoint_index_);
      if (current_waypoint_index_ >= trajectory_.num_waypoints() - 1) {
        if (!trajectory_.IsLoopingEnabled()) {
          // The trajectory is not set to loop.
          StopTrajectory();
        }
      }
      break;
    }
  }
}
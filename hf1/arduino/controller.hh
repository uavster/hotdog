template<typename TrajectoryViewType>
TrajectoryController<TrajectoryViewType>::TrajectoryController(const float run_period_seconds)
  : Controller(run_period_seconds),
    is_started_(false),
    current_waypoint_index_(0) {}

template<typename TrajectoryViewType>
void TrajectoryController<TrajectoryViewType>::trajectory(const TrajectoryViewType &trajectory) {
  trajectory_ = trajectory;
  current_waypoint_index_ = 0;
  is_started_ = false;
}

template<typename TrajectoryViewType>
void TrajectoryController<TrajectoryViewType>::StartTrajectory() {
  current_waypoint_index_ = 0;
  start_seconds_ = GetTimerSeconds();
  is_started_ = true;
}

template<typename TrajectoryViewType>
void TrajectoryController<TrajectoryViewType>::StopTrajectory() {
  is_started_ = false;
}

template<typename TrajectoryViewType>
void TrajectoryController<TrajectoryViewType>::Update(const TimerSecondsType now_seconds) {
  if (!is_started_) { return; }
  const TimerSecondsType seconds_since_start = now_seconds - start_seconds_;
  const auto maybe_index = trajectory_.FindWaypointIndexBeforeSeconds(seconds_since_start, current_waypoint_index_);
  if (!maybe_index.ok()) { return; }
  current_waypoint_index_ = *maybe_index;
  if (!trajectory_.IsLoopingEnabled() && current_waypoint_index_ == trajectory_.num_waypoints() - 1) {
    StopTrajectory();
    return;
  }

  Update(seconds_since_start, current_waypoint_index_);
}
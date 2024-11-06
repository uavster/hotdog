template<typename TState>
TrajectoryView<TState> &TrajectoryView<TState>::EnableLooping(TimerSecondsType after_seconds) { 
  if (num_waypoints_ == 0) { 
    loop_at_seconds_ = -1;
  } else {
    loop_at_seconds_ = waypoints_[num_waypoints_ - 1].seconds() + after_seconds;
  }
  return *this;
}

template<typename TState>
TrajectoryView<TState> &TrajectoryView<TState>::DisableLooping() { 
  loop_at_seconds_ = -1;
  return *this;
}

template<typename TState>
bool TrajectoryView<TState>::IsLoopingEnabled() const { 
  return loop_at_seconds_ >= 0;
}

template<typename TState>
StatusOr<int> TrajectoryView<TState>::FindWaypointIndexBeforeSeconds(TimerSecondsType seconds, int prev_result_index) const {
  if (num_waypoints_ == 0 || seconds < this->seconds(0)) { return Status::kUnavailableError; }
  int i = prev_result_index;
  while((IsLoopingEnabled() || i < num_waypoints_) && this->seconds(i) < seconds) { ++i; }
  return i - 1;
}

template<typename TState>
float TrajectoryView<TState>::seconds(int index) const {
  if (IsLoopingEnabled()) {
    const int normalized_index = IndexMod(index, num_waypoints_);
    const TimerSecondsType prev_loops_seconds = (loop_at_seconds_ - waypoints_[0].seconds()) * (index / num_waypoints_);
    return waypoints_[normalized_index].seconds() + prev_loops_seconds;
  } else {
    ASSERT(index >= 0);
    ASSERT(index < num_waypoints_);
    return waypoints_[index].seconds();
  }
}

template<typename TState>
const TState &TrajectoryView<TState>::state(int index) const {
  if (IsLoopingEnabled()) {
    return waypoints_[IndexMod(index, num_waypoints_)].state();
  } else {
    return waypoints_[min(index, num_waypoints_ - 1)].state();
  }
}

template<typename TState>
TState TrajectoryView<TState>::derivative(int order, int index) const {
  if (order == 0) {
    return state(index);
  } else {
    return (derivative(order - 1, index + 1) - derivative(order - 1, index)) / (seconds(index + 1) - seconds(index));
  }
}

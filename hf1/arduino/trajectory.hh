#include <math.h>

static int compare_seconds(const void *a, const void *b) {
  return ceilf(*reinterpret_cast<const TimerSecondsType *>(a) - *reinterpret_cast<const TimerSecondsType *>(b));
}

template<typename TState>
TrajectoryView<TState>::TrajectoryView(int num_waypoints, const Waypoint<TState> *waypoints)
  : num_waypoints_(num_waypoints), waypoints_(waypoints), loop_after_seconds_(-1) {
  if (num_waypoints > 0) {
    ASSERTM(IsTrajectoryValid(num_waypoints, waypoints), "Two waypoints defined for the same time.");
  }
}

template<typename TState>
bool TrajectoryView<TState>::IsTrajectoryValid(int num_waypoints, const Waypoint<TState> *waypoints) {
    // Check if there are more than one waypoint for the same time.
  TimerSecondsType sorted_waypoint_seconds[num_waypoints];
  for (int i = 0; i < num_waypoints; ++i) { sorted_waypoint_seconds[i] = waypoints[i].seconds(); }
  qsort(sorted_waypoint_seconds, num_waypoints, sizeof(sorted_waypoint_seconds[0]), &compare_seconds);
  for (int i = 0; i < num_waypoints - 1; ++i) {
    if (sorted_waypoint_seconds[i] == sorted_waypoint_seconds[i + 1]) {
      return false;
    }
  }
  return true;
}

template<typename TState>
TrajectoryView<TState> &TrajectoryView<TState>::EnableLooping(TimerSecondsType after_seconds) {
  ASSERTM(after_seconds > 0, "The state cannot go back from the last to the first waypoint in no time.");
  if (num_waypoints_ == 0) { 
    loop_after_seconds_ = -1;
  } else {
    loop_after_seconds_ = after_seconds;
  }
  return *this;
}

template<typename TState>
TrajectoryView<TState> &TrajectoryView<TState>::DisableLooping() { 
  loop_after_seconds_ = -1;
  return *this;
}

template<typename TState>
bool TrajectoryView<TState>::IsLoopingEnabled() const { 
  return loop_after_seconds_ >= 0;
}

template<typename TState>
StatusOr<TimerSecondsType> TrajectoryView<TState>::SecondsBetweenLoops() const {
  if (!IsLoopingEnabled()) {
    return Status::kUnavailableError;
  }
  return loop_after_seconds_;
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
  const TimerSecondsType trajectory_duration = waypoints_[num_waypoints_ - 1].seconds() - waypoints_[0].seconds();
  // If looping is enabled the time to get back to the initial state was given.
  // If looping is not enabled, in order to enable derivative computation, assume that time
  // will be the average time between waypoints.
  const TimerSecondsType loop_after_seconds = loop_after_seconds_ >= 0 ? loop_after_seconds_ : trajectory_duration / num_waypoints_;
  const int normalized_index = IndexMod(index, num_waypoints_);
  const TimerSecondsType prev_loops_seconds = (trajectory_duration + loop_after_seconds) * (index / num_waypoints_);
  return waypoints_[normalized_index].seconds() + prev_loops_seconds;
}

template<typename TState>
const TState &TrajectoryView<TState>::state(int index) const {
  return waypoints_[IndexMod(index, num_waypoints_)].state();
}

template<typename TState>
TState TrajectoryView<TState>::derivative(int order, int index) const {
  if (order == 0) {
    return state(index);
  } else {
    const TimerSecondsType time_interval = seconds(index + 1) - seconds(index);
    ASSERT(time_interval > 0);
    return (derivative(order - 1, index + 1) - derivative(order - 1, index)) / time_interval;
  }
}

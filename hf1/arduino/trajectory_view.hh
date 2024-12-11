namespace {
template<typename TState>
static Waypoint<TState> Bezier(const Waypoint<TState> &a, const Waypoint<TState> &b, const Waypoint<TState> &c, const Waypoint<TState> &d, const float t) {
  const auto a1 = a * (1 - t) + b * t;
  const auto a2 = b * (1 - t) + c * t;
  const auto a3 = c * (1 - t) + d * t;
  const auto b1 = a1 * (1 - t) + a2 * t;
  const auto b2 = a2 * (1 - t) + a3 * t;
  return b1 * (1 - t) + b2 * t;
}

template<typename TState>
static Waypoint<TState> CentripetalCatmullRom(const Waypoint<TState> &p0, const Waypoint<TState> &p1, const Waypoint<TState> &p2, const Waypoint<TState> &p3, const float t) {
  const auto d1 = p0.state().DistanceFrom(p1.state());
  const auto d2 = p1.state().DistanceFrom(p2.state());
  const auto d3 = p2.state().DistanceFrom(p3.state());
  const auto b0 = p1;
  const auto b1 = p1 + (p2 * d1 - p0 * d2 + p1 * (d2 - d1)) / (3 * d1 + 3 * std::sqrt(d1 * d2));
  const auto b2 = p2 + (p1 * d3 - p3 * d2 + p2 * (d2 - d3)) / (3 * d3 + 3 * std::sqrt(d2 * d3));
  const auto b3 = p2;
  return Bezier(b0, b1, b2, b3, t);
}
}

template<typename TState>
TrajectoryView<TState>::TrajectoryView(const TrajectoryInterface<TState> *trajectory)
  : trajectory_(ASSERT_NOT_NULL(trajectory)),
    interpolation_config_(InterpolationConfig{ .type = kNone }),
    loop_after_seconds_(-1) {}

template<typename TState>
Waypoint<TState> TrajectoryView<TState>::GetPeriodicWaypoint(int index) const {
  float lap_duration = LapDuration();
  // If looping is not enabled, in order to enable derivative computation, assume that time
  // will be the average time between waypoints.
  if (loop_after_seconds_ < 0) {
    const TimerSecondsType waypoints_duration = (*trajectory_)[trajectory_->size() - 1].seconds() - (*trajectory_)[0].seconds();
    lap_duration += waypoints_duration / (trajectory_->size() - 1);
  }
  const int num_completed_laps = index / trajectory_->size();
  const auto normalized_index = IndexMod(index, trajectory_->size());
  const auto waypoint_time = (*trajectory_)[normalized_index].seconds() + lap_duration * num_completed_laps;  
  const auto &waypoint_state = (*trajectory_)[normalized_index].state();
  return Waypoint<TState>(waypoint_time, waypoint_state);  
}

template<typename TState>
Waypoint<TState> TrajectoryView<TState>::GetWaypoint(float seconds) const {
  ASSERT_NOT_NULL(trajectory_);
  switch (interpolation_config_.type) {
    case kNone:
      return GetPeriodicWaypoint(trajectory_->FindWaypointAtOrBeforeSeconds(seconds));
    case kLinear:
      {
        const auto periodic_seconds = IndexModf(seconds - (*trajectory_)[0].seconds(), LapDuration()) + (*trajectory_)[0].seconds();
        const int i1 = trajectory_->FindWaypointAtOrBeforeSeconds(periodic_seconds);
        const Waypoint<TState> &w1 = (*trajectory_)[i1];
        const Waypoint<TState> w2 = GetPeriodicWaypoint(i1 + 1);
        const float t = (periodic_seconds - w1.seconds()) / (w2.seconds() - w1.seconds());
        return Waypoint<TState>(seconds, w1.state() * (1 - t) + w2.state() * t);
      }
    case kCubic:
      {
        const int i1 = trajectory_->FindWaypointAtOrBeforeSeconds(seconds);
        Waypoint<TState> w1 = GetPeriodicWaypoint(i1);
        Waypoint<TState> w2 = GetPeriodicWaypoint(i1 + 1);
        Waypoint<TState> w0;
        Waypoint<TState> w3;
        const int i0 = i1 - 1;
        const int i3 = i1 + 2;
        const float t = seconds - (*trajectory_)[i1].seconds();
        
        if (i0 >= 0) {
          w0 = GetPeriodicWaypoint(i0);
        } else {
          // First lap: the previous waypoint is on the line passing over the first two
          // waypoints, before them.
          w0 = Waypoint<TState>(w1.seconds() - 3 * (w2.seconds() - w1.seconds()), w1.state() + (w1.state() - w2.state()) * 3);
        }

        if (IsLoopingEnabled() || i3 < trajectory_->size()) {
          // If trajectory loops, all waypoints repeat cyclically.
          w3 = GetPeriodicWaypoint(i3);
        } else {
          // Last lap: last waypoint is on the line passing over the last two waypoints, 
          // after them.
          w3 = Waypoint<TState>(w2.seconds() + 3 * (w2.seconds() - w1.seconds()), w2.state() + (w2.state() - w1.state()) * 3);
        }

        return CentripetalCatmullRom(w0, w1, w2, w3, t);
      }
  }
  return Waypoint<TState>();  // Avoid compiler warning.
}

template<typename TState>
TrajectoryView<TState> &TrajectoryView<TState>::EnableInterpolation(const InterpolationConfig &config) {
  interpolation_config_ = config;
  return *this;
}

template<typename TState>
TrajectoryView<TState> &TrajectoryView<TState>::DisableInterpolation() {
  interpolation_config_.type = kNone;
  return *this;
}

template<typename TState>
TrajectoryView<TState> &TrajectoryView<TState>::EnableLooping(TimerSecondsType after_seconds) {
  ASSERT_NOT_NULL(trajectory_);
  ASSERTM(after_seconds > 0, "The state cannot go back from the last to the first waypoint in no time.");
  if (trajectory_->size() == 0) {
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
float TrajectoryView<TState>::LapDuration() const {
  ASSERT_NOT_NULL(trajectory_);
  TimerSecondsType duration = (*trajectory_)[trajectory_->size() - 1].seconds() - (*trajectory_)[0].seconds();
  // If looping is enabled, the time to get back to the initial state is part of a lap.
  if (loop_after_seconds_ >= 0) {
    duration += loop_after_seconds_;
  }
  return duration;
}

template<typename TState>
TState TrajectoryViewInterface<TState>::state(float seconds) const {
  return GetWaypoint(seconds).state();
}

template<typename TState>
TState TrajectoryViewInterface<TState>::derivative(int order, float seconds, float epsilon) const {
  if (order == 0) {
    return state(seconds);
  } else {
    return (derivative(order - 1, seconds + epsilon) - derivative(order - 1, seconds)) / epsilon;
  }
}

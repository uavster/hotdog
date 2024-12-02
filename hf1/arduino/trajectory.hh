#include "logger_interface.h"

namespace {

static int compare_seconds(const void *a, const void *b) {
  return ceilf(*reinterpret_cast<const TimerSecondsType *>(a) - *reinterpret_cast<const TimerSecondsType *>(b));
}

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

} // namespace

template<typename TState>
TrajectoryView<TState>::TrajectoryView(int num_waypoints, const Waypoint<TState> *waypoints)
  : num_waypoints_(num_waypoints),
    waypoints_(waypoints),
    interpolation_config_(InterpolationConfig{ .type = kNone, .num_sampling_points = 0 }),
    loop_after_seconds_(-1) {
  if (num_waypoints > 0) {
    ASSERTM(IsTrajectoryValid(num_waypoints, waypoints), "Two waypoints defined for the same time.");
  }
}

template<typename TState>
int TrajectoryView<TState>::NumWaypoints() const {
  if (interpolation_config_.type == kNone) {
    return num_waypoints_;
  } else {
    return interpolation_config_.num_sampling_points;
  }
}

template<typename TState>
Waypoint<TState> TrajectoryView<TState>::GetPeriodicWaypoint(int index) const {
  float lap_duration = LapDuration();
  // If looping is not enabled, in order to enable derivative computation, assume that time
  // will be the average time between waypoints.
  if (loop_after_seconds_ < 0) {
    const TimerSecondsType waypoints_duration = waypoints_[num_waypoints_ - 1].seconds() - waypoints_[0].seconds();
    lap_duration += waypoints_duration / (num_waypoints_ - 1);
  }
  const int num_completed_laps = index / num_waypoints_;
  const auto normalized_index = IndexMod(index, num_waypoints_);
  const auto waypoint_time = waypoints_[normalized_index].seconds() + lap_duration * num_completed_laps;  
  const auto &waypoint_state = waypoints_[normalized_index].state();
  return Waypoint<TState>(waypoint_time, waypoint_state);  
}

template<typename TState>
Waypoint<TState> TrajectoryView<TState>::GetWaypoint(int index) const {
  switch (interpolation_config_.type) {
    case kNone:
      return GetPeriodicWaypoint(index);
    case kLinear:
      {
        const int num_lap_waypoints = IsLoopingEnabled() ? num_waypoints_ : num_waypoints_ - 1;
        const float remapped_index = (index * num_lap_waypoints) / (static_cast<float>(interpolation_config_.num_sampling_points) - 1);
        const int i1 = static_cast<int>(remapped_index);
        const int i2 = i1 + 1;
        const float t = remapped_index - i1;
        return GetPeriodicWaypoint(i1) * (1 - t) + GetPeriodicWaypoint(i2) * t;
      }
    case kCubic:
      {
        const int num_lap_waypoints = IsLoopingEnabled() ? num_waypoints_ : num_waypoints_ - 1;
        const float remapped_index = (index * num_lap_waypoints) / (static_cast<float>(interpolation_config_.num_sampling_points) - 1);
        const int i1 = static_cast<int>(remapped_index);
        Waypoint<TState> w1 = GetPeriodicWaypoint(i1);
        Waypoint<TState> w2 = GetPeriodicWaypoint(i1 + 1);
        Waypoint<TState> w0;
        Waypoint<TState> w3;
        const int i0 = i1 - 1;
        const int i3 = i1 + 2;
        const float t = remapped_index - i1;
        
        if (i0 >= 0) {
          w0 = GetPeriodicWaypoint(i0);
        } else {
          // First lap: the previous waypoint is on the line passing over the first two
          // waypoints, before them.
          w0 = Waypoint<TState>(w1.seconds() - 3 * (w2.seconds() - w1.seconds()), w1.state() + (w1.state() - w2.state()) * 3);
        }

        if (IsLoopingEnabled() || i3 < num_waypoints_) {
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
  while (this->seconds(i) < seconds) { ++i; }
  return i - 1;
}

template<typename TState>
float TrajectoryView<TState>::LapDuration() const {
  TimerSecondsType duration = waypoints_[num_waypoints_ - 1].seconds() - waypoints_[0].seconds();
  // If looping is enabled, the time to get back to the initial state is part of a lap.
  if (loop_after_seconds_ >= 0) {
    duration += loop_after_seconds_;
  }
  return duration;
}

template<typename TState>
float TrajectoryView<TState>::seconds(int index) const {
  return GetWaypoint(index).seconds();
}

template<typename TState>
TState TrajectoryView<TState>::state(int index) const {
  return GetWaypoint(index).state();
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

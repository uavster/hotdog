#include "trajectory.h"

template<typename TState>
TrajectoryController<TState>::TrajectoryController(const char *name, const float run_period_seconds)
  : Controller(name, run_period_seconds), trajectory_(NULL) {}

template<typename TState>
void TrajectoryController<TState>::trajectory(const TrajectoryViewInterface<TState> *trajectory) {
  trajectory_ = trajectory;
}

template<typename TState>
void TrajectoryController<TState>::Start() {
  Controller::Start();
  seconds_since_start_ = 0;
}

template<typename TState>
float TrajectoryController<TState>::NumCompletedLaps() const {
  if (IsTrajectoryFinished() && seconds_since_start_ > trajectory_->LapDuration()) {
    return 1;
  }
  return seconds_since_start_ / trajectory_->LapDuration();
}

template<typename TState>
bool TrajectoryController<TState>::IsTrajectoryFinished() const {
  return !trajectory_->IsLoopingEnabled() && 
          seconds_since_start_ > trajectory_->LapDuration();
}

template<typename TState>
void TrajectoryController<TState>::Update(const TimerSecondsType seconds_since_start) {
  seconds_since_start_ = seconds_since_start;
  if (IsTrajectoryFinished()) {
    Stop();
  }
}
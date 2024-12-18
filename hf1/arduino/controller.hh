#include "trajectory.h"

template<typename TState>
TrajectoryController<TState>::TrajectoryController(const char *name, const float run_period_seconds)
  : Controller(name, run_period_seconds), trajectory_(NULL) {}

template<typename TState>
void TrajectoryController<TState>::trajectory(const TrajectoryViewInterface<TState> *trajectory) {
  trajectory_ = trajectory;
}

template<typename TState>
void TrajectoryController<TState>::Update(const TimerSecondsType seconds_since_start) {
  if (!trajectory_->IsLoopingEnabled() && 
      seconds_since_start > trajectory_->LapDuration()) {
    Stop();
  }
}
#include "trajectory.h"

template<typename TrajectoryViewType>
TrajectoryController<TrajectoryViewType>::TrajectoryController(const char *name, const float run_period_seconds)
  : Controller(name, run_period_seconds) {}

template<typename TrajectoryViewType>
void TrajectoryController<TrajectoryViewType>::trajectory(const TrajectoryViewType &trajectory) {
  trajectory_ = trajectory;
}

template<typename TrajectoryViewType>
void TrajectoryController<TrajectoryViewType>::Update(const TimerSecondsType seconds_since_start) {
  if (!trajectory_.IsLoopingEnabled() && 
      seconds_since_start > trajectory_.LapDuration()) {
    Stop();
  }
}
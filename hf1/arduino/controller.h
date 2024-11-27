#ifndef CONTROLLER_INCLUDED_
#define CONTROLLER_INCLUDED_

#include "periodic_runnable.h"

class Controller : public PeriodicRunnable {
public:
  explicit Controller(float run_period_seconds);

protected:
  virtual void Update(TimerSecondsType now_seconds) = 0;

private:
  virtual void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override;
};

template<typename TrajectoryViewType>
class TrajectoryController : public Controller {
public:
  TrajectoryController(float run_period_seconds);
  
  void trajectory(const TrajectoryViewType &trajectory);
  const TrajectoryViewType &trajectory() const { return trajectory_; }

  void StartTrajectory();
  void StopTrajectory();

  using ControllerState = enum { kStopped, kFollowingTrajectory };
  ControllerState state() const { return state_; }

protected:
  // Subclasses must override this function to update controls.
  virtual void Update(TimerSecondsType seconds_since_start, int current_waypoint_index) = 0;
  // Subclasses must override this function to stop movement.
  virtual void Stop() = 0;

private:
  virtual void Update(TimerSecondsType now_seconds) override;

  TrajectoryViewType trajectory_;
  bool is_started_;
  TimerSecondsType start_seconds_;
  int current_waypoint_index_;
  ControllerState state_;
};

#include "controller.hh"

#endif  // CONTROLLER_INCLUDED_
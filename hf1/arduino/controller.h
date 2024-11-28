#ifndef CONTROLLER_INCLUDED_
#define CONTROLLER_INCLUDED_

#include "periodic_runnable.h"

// Generic controller.
//
// It calls Update() periodically, with the period specified by `run_period_seconds`.
//
// Controllers must subclass it and update controls in their overridden Update() function.
class Controller : public PeriodicRunnable {
public:
  explicit Controller(float run_period_seconds);

protected:
  virtual void Update(TimerSecondsType now_seconds) = 0;

private:
  virtual void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override;
};

// Generic trajectory controller.
//
// It calls Update() periodically with the target waypoint to reach next. At the last 
// waypoint's time, it calls Stop().

// When calling Update(), it does not take into account the time that the plant will take 
// to reach the target, i.e. it assumes the waypoint's state can be reached instantaneously. 
// For instance, a waypoint with state S at time T seconds, will be passed to Update() 
// T seconds after the trajectory started.
//
// Because of that plant latency, there will be a time error between the actual trajectory 
// and the passed trajectory. This can be mitigated in two ways:
// - The subclassed controller can look ahead the next waypoints and start moving earlier.
// - The caller can upsample the passed trajectory with interpolation, using a sampling time 
//   long enough for the controller to take the plant from one waypoint's state to the next
//   (see interpolation capabilities of the Trajectory class).
//
// Trajectory controllers with these semantics should subclass TrajectoryControler, update 
// the controls in their overridden Update() function, and stop the plant in their
// overridden Stop().
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
  virtual void Update(TimerSecondsType seconds_since_start, int target_waypoint_index) = 0;
  // Subclasses must override this function to stop movement.
  virtual void Stop() = 0;

private:
  virtual void Update(TimerSecondsType now_seconds) override;

  TrajectoryViewType trajectory_;
  bool is_started_;
  TimerSecondsType start_seconds_;
  int target_waypoint_index_;
  ControllerState state_;
};

#include "controller.hh"

#endif  // CONTROLLER_INCLUDED_
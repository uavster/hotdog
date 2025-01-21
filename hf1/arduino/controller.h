#ifndef CONTROLLER_INCLUDED_
#define CONTROLLER_INCLUDED_

#include "periodic_runnable.h"
#include "trajectory_view.h"

// Generic controller.
//
// It calls Update() periodically, with the period specified by `run_period_seconds`.
//
// Controllers must subclass it and update controls in their overridden Update() function.
class Controller : public PeriodicRunnable {
public:
  explicit Controller(const char *name, float run_period_seconds);
  virtual void Start();
  virtual void Stop();

  bool is_started() const { return is_started_; }

protected:
  virtual void Update(TimerSecondsType seconds_since_start) = 0;
  // Subclasses may override this function if any special action is required to stop the plant.
  virtual void StopControl() {};

private:
  virtual void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override;

  bool is_started_;
  TimerSecondsType start_seconds_;
};

// Generic trajectory controller.
//
// It calls Update() periodically with the time since Start() was called. At the last 
// waypoint's time, it calls Stop().

// When calling Update(), this class does not take into account the time that the plant will take 
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
// overridden StopControl().
template<typename TState>
class TrajectoryController : public Controller {
public:
  TrajectoryController(const char *name, float run_period_seconds);
  
  // Does not take ownership of the pointee, which must outlive this object.
  void trajectory(const TrajectoryViewInterface<TState> *trajectory);
  const TrajectoryViewInterface<TState> &trajectory() const { return *ASSERT_NOT_NULL(trajectory_); }

  virtual void Start() override;
  
  float NumCompletedLaps() const;
  bool IsTrajectoryFinished() const;

protected:
  // Subclasses must override this function to update controls.
  // The parent's method must always be called.
  virtual void Update(TimerSecondsType seconds_since_start) override;

private:  
  const TrajectoryViewInterface<TState> *trajectory_;
  TimerSecondsType seconds_since_start_;
};

#include "controller.hh"

#endif  // CONTROLLER_INCLUDED_
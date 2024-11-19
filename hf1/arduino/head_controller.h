#ifndef HEAD_CONTROLLER_
#define HEAD_CONTROLLER_

#include "controller.h"
#include "head_state.h"
#include "trajectory.h"

using HeadTargetState = State<HeadStateVars, 0>;

// Defines the state of the robot's head at a given time. The controller class decides what
// part of the state to use. For instance, some controllers may ignore the time.
using HeadWaypoint = Waypoint<HeadTargetState>;

// A view of a collection of head waypoints.
// The view does not own the memory containing the waypoints, so they must outlive any 
// view objects referencing them.
using HeadTrajectoryView = TrajectoryView<HeadTargetState>;

class HeadTrajectoryController : public TrajectoryController<HeadTrajectoryView> {
public:
  HeadTrajectoryController();

protected:
  virtual void Update(TimerSecondsType seconds_since_start, int current_waypoint_index) override;
  virtual void Stop() override {};
};

#endif  // HEAD_CONTROLLER_
#ifndef BASE_TRAJECTORIES_INCLUDED_
#define BASE_TRAJECTORIES_INCLUDED_

#include "base_state.h"
#include "trajectory_view.h"
#include "modulated_trajectory_view.h"

// Defines the state of the robot's base at a given time. The controller class decides what
// part of the state to use. For instance, some controllers may ignore the time and/or the 
// yaw angle.
using BaseWaypoint = Waypoint<BaseTargetState>;

// A view of a collection of base waypoints.
// The view does not own the memory containing the waypoints, so they must outlive any 
// view objects referencing them.
using BaseTrajectoryView = TrajectoryView<BaseTargetState>;

class BaseModulatedTrajectoryView : public ModulatedTrajectoryView<BaseTargetState> {
public:
  // Returns the waypoint at the given index, after applying interpolation.
  BaseWaypoint GetWaypoint(float seconds) const override;
};

#endif  // BASE_TRAJECTORIES_INCLUDED_
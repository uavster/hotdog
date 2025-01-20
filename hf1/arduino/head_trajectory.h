#ifndef HEAD_TRAJECTORIES_INCLUDED_
#define HEAD_TRAJECTORIES_INCLUDED_

#include "head_state.h"
#include "modulated_trajectory_view.h"
#include "trajectory.h"
#include "trajectory_view.h"
#include "mixed_trajectory_view.h"

// Defines the state of the robot's head at a given time. The controller class decides what
// part of the state to use. For instance, some controllers may ignore the time.
using HeadWaypoint = Waypoint<HeadTargetState>;

// A view of a collection of head waypoints.
// The view does not own the memory containing the waypoints, so they must outlive any 
// view objects referencing them.
using HeadTrajectoryView = TrajectoryView<HeadTargetState>;

class HeadModulatedTrajectoryView : public ModulatedTrajectoryView<HeadTargetState> {
public:
  // Returns the waypoint at the given index, after applying interpolation.
  HeadWaypoint GetWaypoint(float seconds) const override;
};

using HeadMixedTrajectoryView = MixedTrajectoryView<HeadTargetState>;

#endif  // HEAD_TRAJECTORIES_INCLUDED_
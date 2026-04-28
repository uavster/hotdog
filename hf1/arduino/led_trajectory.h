#ifndef LED_TRAJECTORIES_INCLUDED_
#define LED_TRAJECTORIES_INCLUDED_

#include "color_state.h"
#include "modulated_trajectory_view.h"
#include "trajectory.h"
#include "trajectory_view.h"
#include "mixed_trajectory_view.h"

// Defines the state of the robot's head at a given time. The controller class decides what
// part of the state to use. For instance, some controllers may ignore the time.
using LedHSVWaypoint = Waypoint<ColorHSVTargetState>;

// A view of a collection of head waypoints.
// The view does not own the memory containing the waypoints, so they must outlive any 
// view objects referencing them.
using LedHSVTrajectoryView = TrajectoryView<ColorHSVTargetState>;

class LedHSVModulatedTrajectoryView : public ModulatedTrajectoryView<ColorHSVTargetState> {
public:
  // Returns the waypoint at the given index, after applying interpolation.
  LedHSVWaypoint GetWaypoint(float seconds) const override;
};

using LedHSVMixedTrajectoryView = MixedTrajectoryView<ColorHSVTargetState>;

#endif  // LED_TRAJECTORIES_INCLUDED_
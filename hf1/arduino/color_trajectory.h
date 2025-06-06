#ifndef COLOR_TRAJECTORY_INCLUDED_
#define COLOR_TRAJECTORY_INCLUDED_

#include "trajectory.h"
#include "trajectory_view.h"
#include "mixed_trajectory_view.h"
#include "envelope_trajectory.h"
#include "modulated_trajectory_view.h"
#include "color_state.h"

// Defines a color in HSV space at a given time.
using ColorHSVWaypoint = Waypoint<ColorHSVTargetState>;

// A view of a collection of HSV color waypoints.
// The view does not own the memory containing the waypoints, so they must outlive any 
// view objects referencing them.
using ColorHSVTrajectoryView = TrajectoryView<ColorHSVTargetState>;

class ColorHSVModulatedTrajectoryView : public ModulatedTrajectoryView<ColorHSVTargetState> {
public:
  // Returns the waypoint at the given index, after applying interpolation.
  ColorHSVWaypoint GetWaypoint(float seconds) const override;
};

using ColorHSVMixedTrajectoryView = MixedTrajectoryView<ColorHSVTargetState>;

#endif  // COLOR_TRAJECTORY_INCLUDED_
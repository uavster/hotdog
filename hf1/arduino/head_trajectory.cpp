#include "head_trajectory.h"

HeadWaypoint HeadModulatedTrajectoryView::GetWaypoint(float seconds) const {
  // Modulate the carrier with the enveloped modulator.
  // Transform the modulator with the carrier.
  const auto angle_modulator = modulator().state(seconds).location() * envelope().state(seconds).location().amplitude();
  const auto carrier_angle = carrier().state(seconds).location();
  return HeadWaypoint(
    /*seconds=*/seconds, 
    HeadTargetState({
      HeadStateVars(
        carrier_angle.pitch() + angle_modulator.pitch(),
        carrier_angle.roll() + angle_modulator.roll()
      )
    })
  );
}

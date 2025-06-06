#include "color_trajectory.h"

ColorHSVWaypoint ColorHSVModulatedTrajectoryView::GetWaypoint(float seconds) const {
  // Modulate the carrier with the enveloped modulator.
  // Transform the modulator with the carrier.
  const auto color_modulator = modulator().state(seconds).location() * envelope().state(seconds).location().amplitude();
  const auto carrier_color = carrier().state(seconds).location();
  return ColorHSVWaypoint(
    /*seconds=*/seconds, 
    ColorHSVTargetState({ carrier_color + color_modulator })
  );
}

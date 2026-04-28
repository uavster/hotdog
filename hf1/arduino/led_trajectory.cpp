#include "led_trajectory.h"

LedHSVWaypoint LedHSVModulatedTrajectoryView::GetWaypoint(float seconds) const {
  // Modulate the carrier with the enveloped modulator.
  // Transform the modulator with the carrier.
  const auto color_modulator = modulator().state(seconds).location() * envelope().state(seconds).location().amplitude();
  const auto color_angle = carrier().state(seconds).location();
  return LedHSVWaypoint(
    /*seconds=*/seconds, 
    ColorHSVTargetState({
      ColorHSVStateVars(
        ColorHSV(
          color_angle.hsv().hue() + color_modulator.hsv().hue(),
          color_angle.hsv().saturation() + color_modulator.hsv().saturation(),
          color_angle.hsv().value() + color_modulator.hsv().value()
        )
      )
    })
  );
}

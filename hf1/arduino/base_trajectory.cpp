#include "base_trajectory.h"

#define kPositionDiffLookAheadSeconds 0.03

BaseWaypoint BaseModulatedTrajectoryView::GetWaypoint(float seconds) const {
  // Modulate the carrier with the enveloped modulator.
  // Transform the modulator with the carrier.
  const auto carrier_pos = carrier().state(seconds).location().position();
  const auto carrier_pos_diff = carrier().state(seconds + kPositionDiffLookAheadSeconds).location().position() - carrier_pos;
  const float carrier_angle = -atan2f(carrier_pos_diff.y, carrier_pos_diff.x);
  const float cos_angle = cosf(carrier_angle);
  const float sin_angle = sinf(carrier_angle);
  const float envelope_value = envelope().state(seconds).location().amplitude();
  const auto modulator_pos = modulator().state(seconds).location().position() * envelope_value;
  return BaseWaypoint(
    /*seconds=*/seconds, 
    BaseTargetState({
      BaseStateVars(
        Point(
          modulator_pos.x * cos_angle + modulator_pos.y * sin_angle + carrier_pos.x, 
          modulator_pos.y * cos_angle - modulator_pos.x * sin_angle + carrier_pos.y
        ), 
        /*yaw=*/0
      )
    })
  );
}
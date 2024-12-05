#include "head_controller.h"
#include "servos.h"

#define kHeadTrajeactoryControllerLoopPeriodSeconds 0.03

HeadWaypoint HeadModulatedTrajectoryView::GetWaypoint(int index) const {
  // Modulate the carrier with the enveloped modulator.
  // Transform the modulator with the carrier.
  const auto angle_modulator = modulator().state(index).location() * envelope().state(index).location().amplitude();
  return HeadWaypoint(
    /*seconds=*/index * carrier().interpolation_config().sampling_period_seconds, 
    HeadTargetState({
      HeadStateVars(
        carrier().state(index).location().pitch() + angle_modulator.pitch(),
        carrier().state(index).location().roll() + angle_modulator.roll()
      )
    })
  );
}

HeadTrajectoryController::HeadTrajectoryController(const char *name) 
  : TrajectoryController<HeadModulatedTrajectoryView>(name, kHeadTrajeactoryControllerLoopPeriodSeconds) {}

void HeadTrajectoryController::Update(TimerSecondsType seconds_since_start, int current_waypoint_index) {
  TimerSecondsType time_fraction = (seconds_since_start - trajectory().seconds(current_waypoint_index)) / (trajectory().seconds(current_waypoint_index + 1) - trajectory().seconds(current_waypoint_index));
  const State ref_position = trajectory().state(current_waypoint_index) + time_fraction * (trajectory().state(current_waypoint_index + 1) - trajectory().state(current_waypoint_index));
  SetHeadPitchDegrees(DegreesFromRadians(ref_position.location().pitch()));
  SetHeadRollDegrees(DegreesFromRadians(ref_position.location().roll()));
}
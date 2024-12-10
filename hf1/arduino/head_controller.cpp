#include "head_controller.h"
#include "servos.h"

#define kHeadTrajeactoryControllerLoopPeriodSeconds 0.03

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

HeadTrajectoryController::HeadTrajectoryController(const char *name) 
  : TrajectoryController<HeadModulatedTrajectoryView>(name, kHeadTrajeactoryControllerLoopPeriodSeconds) {}

void HeadTrajectoryController::Update(TimerSecondsType seconds_since_start) {
  TrajectoryController<HeadModulatedTrajectoryView>::Update(seconds_since_start);
  
  const State ref_position = trajectory().state(seconds_since_start);
  SetHeadPitchDegrees(DegreesFromRadians(ref_position.location().pitch()));
  SetHeadRollDegrees(DegreesFromRadians(ref_position.location().roll()));
}
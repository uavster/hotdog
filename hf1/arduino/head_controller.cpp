#include "head_controller.h"
#include "servos.h"

#define kHeadTrajeactoryControllerLoopPeriodSeconds 0.03

HeadTrajectoryController::HeadTrajectoryController(const char *name) 
  : TrajectoryController<HeadTargetState>(name, kHeadTrajeactoryControllerLoopPeriodSeconds) {}

void HeadTrajectoryController::Update(TimerSecondsType seconds_since_start) {
  TrajectoryController<HeadTargetState>::Update(seconds_since_start);
  if (!is_started()) { return; }
  
  const State ref_position = trajectory().state(seconds_since_start);
  SetHeadYawDegrees(DegreesFromRadians(ref_position.location().yaw()));
  SetHeadPitchDegrees(DegreesFromRadians(ref_position.location().pitch()));
  SetHeadRollDegrees(DegreesFromRadians(ref_position.location().roll()));
}
#include "head_controller.h"
#include "servos.h"

#define kHeadTrajeactoryControllerLoopPeriodSeconds 0.03

HeadTrajectoryController::HeadTrajectoryController() 
  : TrajectoryController<HeadTrajectoryView>(kHeadTrajeactoryControllerLoopPeriodSeconds) {}

void HeadTrajectoryController::Update(TimerSecondsType seconds_since_start, int current_waypoint_index) {
  TimerSecondsType time_fraction = (seconds_since_start - trajectory().seconds(current_waypoint_index)) / (trajectory().seconds(current_waypoint_index + 1) - trajectory().seconds(current_waypoint_index));
  const State ref_position = trajectory().state(current_waypoint_index) + time_fraction * (trajectory().state(current_waypoint_index + 1) - trajectory().state(current_waypoint_index));
  SetHeadPitchDegrees(DegreesFromRadians(ref_position.location().pitch()));
  SetHeadRollDegrees(DegreesFromRadians(ref_position.location().roll()));
}
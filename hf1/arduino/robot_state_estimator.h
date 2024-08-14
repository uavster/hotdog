#ifndef ROBOT_STATE_ESTIMATOR_
#define ROBOT_STATE_ESTIMATOR_

#include "robot_state.h"

void InitRobotStateEstimator();
void RunRobotStateEstimator();
const RobotState &GetRobotState();

#endif  // ROBOT_STATE_ESTIMATOR_

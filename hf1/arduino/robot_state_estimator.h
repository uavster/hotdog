#ifndef ROBOT_STATE_ESTIMATOR_
#define ROBOT_STATE_ESTIMATOR_

#include "robot_state.h"

void InitRobotStateEstimator();
void RunRobotStateEstimator();
const BaseState &GetBaseState();
void NotifyLeftMotorDirection(TimerTicksType timer_ticks, bool forward);
void NotifyRightMotorDirection(TimerTicksType timer_ticks, bool forward);

#endif  // ROBOT_STATE_ESTIMATOR_

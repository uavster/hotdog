#ifndef ROBOT_STATE_ESTIMATOR_
#define ROBOT_STATE_ESTIMATOR_

#include "base_state.h"
#include "timer.h"

void InitRobotStateEstimator();
void RunRobotStateEstimator();
BaseState GetBaseState();
TimerNanosType GetBaseStateUpdateNanos();
void NotifyLeftMotorDirection(TimerTicksType timer_ticks, bool forward);
void NotifyRightMotorDirection(TimerTicksType timer_ticks, bool forward);

#endif  // ROBOT_STATE_ESTIMATOR_

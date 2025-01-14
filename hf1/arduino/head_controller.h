#ifndef HEAD_CONTROLLER_
#define HEAD_CONTROLLER_

#include "controller.h"
#include "head_state.h"

class HeadTrajectoryController : public TrajectoryController<HeadTargetState> {
public:
  HeadTrajectoryController(const char *name);

protected:
  virtual void Update(TimerSecondsType seconds_since_start) override;
};

#endif  // HEAD_CONTROLLER_
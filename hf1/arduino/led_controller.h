#ifndef LED_CONTROLLER_INCLUDED_
#define LED_CONTROLLER_INCLUDED_

#include "controller.h"
#include "led.h"
#include "color_state.h"

class LedHSVTrajectoryController : public TrajectoryController<ColorHSVTargetState> {
public:
  // Does not take ownership of led_rgb, which must outlive this object.
  LedHSVTrajectoryController(const char *name, LedRGB *led_rgb);

protected:
  virtual void Update(TimerSecondsType seconds_since_start) override;

private:
  LedRGB &led_rgb_;
};

#endif  // LED_CONTROLLER_INCLUDED_
#include "led_controller.h"
#include "led.h"
#include "color_state.h"
#include "logger_interface.h"

constexpr float kLedTrajeactoryControllerLoopPeriodSeconds = 1.0f / 30;

LedHSVTrajectoryController::LedHSVTrajectoryController(const char *name, LedRGB *led_rgb) 
  : TrajectoryController<ColorHSVTargetState>(name, kLedTrajeactoryControllerLoopPeriodSeconds), led_rgb_(*ASSERT_NOT_NULL(led_rgb)) {}

void LedHSVTrajectoryController::Update(TimerSecondsType seconds_since_start) {
  TrajectoryController<ColorHSVTargetState>::Update(seconds_since_start);
  if (!is_started()) { return; }
  
  led_rgb_.SetColor(trajectory().state(seconds_since_start).location().hsv());
}
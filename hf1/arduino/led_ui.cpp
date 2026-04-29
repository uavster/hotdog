#include "led_ui.h"
#include "color_trajectory.h"

namespace {
const ColorRGB kBootingColor = ColorRGB(1, 1, 1);
const ColorRGB kBootErrorColor = ColorRGB(1, 0, 0);
const ColorRGB kConnectingP2PColor = ColorRGB(0, 0, 1);
const ColorRGB kP2PConnectedColor = ColorRGB(0, 1, 0);

constexpr float kShuttingDownBlinkingPeriodS = 0.25f;

const Trajectory<ColorHSVTargetState, /*Capacity=*/2> kShuttingDownColorTrajectory({
  ColorHSVWaypoint(0, ColorHSVTargetState{{ ColorHSV(kRedHue, kLowestSaturation, kDarkestValue) }}),
  ColorHSVWaypoint(kShuttingDownBlinkingPeriodS, ColorHSVTargetState{{ ColorHSV(kGreenHue, kHighestSaturation, kBrightestValue) }})
});
const TrajectoryView<ColorHSVTargetState> kShuttingDownColorTrajectoryView(&kShuttingDownColorTrajectory, InterpolationConfig{ .type = InterpolationType::kNone }, /*loop_after_seconds=*/kShuttingDownBlinkingPeriodS);
} // namespace

LedUI::LedUI(LedHSVTrajectoryController *led_trajectory_controller, LedRGB *led_rgb) 
  : led_trajectory_controller_(*ASSERT_NOT_NULL(led_trajectory_controller)),
    led_rgb_(*ASSERT_NOT_NULL(led_rgb)) {}

void LedUI::SetStatus(LedUI::Status status) const {
  switch(status) {
    // No LED trajectory controller during boot states; set LED directly.
    case LedUI::Status::kBooting: led_rgb_.SetColor(kBootingColor); break;
    case LedUI::Status::kBootError: led_rgb_.SetColor(kBootErrorColor); break;

    case LedUI::Status::kConnectingP2P: led_rgb_.SetColor(kConnectingP2PColor); break;
    case LedUI::Status::kP2PConnected: 
      // Ideally not visible for too long as the action client sets the ok color (green).
      // However, it's useful to see if the action client is connected but not ours.
      led_rgb_.SetColor(kP2PConnectedColor);
      // LED commands are handed over to the action client in this status.      
      // TODO: Enable the LED trajectory execution action.
      break;

    case LedUI::Status::kShuttingDown:
      // TODO: Disable the LED trajectory execution action.
      led_trajectory_controller_.trajectory(&kShuttingDownColorTrajectoryView);
      led_trajectory_controller_.Start();
      break;
  }
}
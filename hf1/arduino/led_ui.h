// This module is responsible for setting the RGB values of the LED 
// to communicate different conditions on the robot. 
// It should be the only module writing to the LED.

#ifndef LED_UI_INCLUDED_
#define LED_UI_INCLUDED_

#include "led_controller.h"

class LedUI {
public:
  // Does not take ownserhip of the pointee, which must outlive this object.
  LedUI(LedHSVTrajectoryController *led_trajectory_controller, LedRGB *led_rgb);

  enum Status {
    // Boot states are set while in setup(). The LED trajectory controller won't be 
    // running by then. The LED should be set directly in those states.
    kBooting, // MCU is booting and configuring all onboard systems.
    kBootError, // An error occured while booting.

    // States after boot that apply during loop() may use the LED trajectory controller.
    kConnectingP2P, // The MCU is connecting with the other end of the P2P link.
    kP2PConnected,  // The P2P link was established: LED control handed over to action interface.

    kShuttingDown // The system is shutting down.
  };
  void SetStatus(LedUI::Status status) const;

private:
  LedHSVTrajectoryController &led_trajectory_controller_;
  LedRGB &led_rgb_;
};

#endif  // LED_UI_INCLUDED_
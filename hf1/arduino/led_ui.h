// This module is responsible for setting the RGB values of the LED 
// to communicate different conditions on the robot. 
// It should be the only module writing to the LED. 

// LED behavior:
// -- Initially, the MCU sets the LED -- 
// MCU boots -> white
// MCU boot error -> red
// MCU boot ok and attempting P2P connection -> blue
// P2P connected -> Jetson drives LED, Jetson sets hostname reported by MCU (must match QR code under robot)

// -- From this point on, the Jetson sets the LED --
// Jetson has wifi -> green 
// Jetson does not have wifi -> blinking blue
// If blinking blue, config takes camera feed and decodes phone's flash to get wifi SSID and password.
//    If wifi ok, static green
//    If wifi error, red for 2 seconds and back to bliking blue

// To pair with phone:
//  The user taps on "pair" in the phone app and scans the QR code under the robot. 
//  The app connects to websocket at `hostname.local` via https and asks for pairing (everything else is still forbidden)
//  The Jetson sets the LED to encode a short random token A 
//  The phone decodes the A token with the camera and sends it back to the Jetson
//  If token matches, the robot knows that the phone is physically in front and the pairing continues -> green
//  If token does not match, pairing is rejected -> 2-second red and back to green
//  If pairing continues, the Jetson then generates a private unique authentication token and sends it back to the phone (via https websocket)
//  The phone stores it and sends it to the Jetson for authorization every time it tries to connect to it.

// In addition to the regular interaction with the robot through the UI, the user can get the CLI command to ssh into the robot from a computer.
// When pairing for the first time with the phone app, we prompt the user to change the password or keep the default one. It must be clearly
// stated that leaving the default password is dangerous and the user must press an "I accept the risk" button to proceed.

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
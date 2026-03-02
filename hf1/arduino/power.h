// Power management functions.
//
// The input power comes from the source with the highest voltage among:
// a) AA battery cells installed in the six internal holders.
// b) 7.2V from the onboard regulator powered by an external power source connected to the barrel jack.
//
// When the power button is pressed, the board outputs power to
// 1) Jetson Nano or Jetson Orin Nano
// 2) Motors
// 3) Servos 
// 
// If (1) is a Jetson Nano, the power is routed through the GPIO header from the robot's PCB.
// Otherwise, if it's a Jetson Orin Nano, the power is routed through the internal barrel jack.
// The latter configuration bypasses the current measure resistor and is not accounted in current
// or power measures.
//
// When the power button is pressed for about 2 seconds, the MCU urges the Jetson to shut down,
// and turns power off when connection is lost or after a timeout period. When the button is pressed
// for more than 6 seconds, the power circuit on the board forces power disconnection.
//
// The PCB can be powered from a USB cable as well, but no power will be output to the Jetson or
// the motors, and the USB current limits will apply, which might result in servo glitches, for 
// instance. The power button has no effect on USB power. USB power is only meant for MCU flashing 
// and MCU-only tests, not for normal robot operation. Power drawn from USB is not included in global 
// current and power measurements returned by the funcions in this header.

#ifndef POWER_INCLUDED_
#define POWER_INCLUDED_

// Cuts power to the control board from the internal batteries or external power source.
// Calling this function is equivalent to pressing the power button until hardware shutdown. 
void PowerOff();

// Returns the voltage of the internal batteries or external power source 
float GetPowerVolts();

// Returns the instantaneous current taken from the internal batteries or external power source, in amps.
// This is the current consumed by the robot's board, the motors, the servos and the Jetson Nano.
// If the robot carries a Jetson Orin Nano, its current is not included in the measure as it goes straight
// into the Jetson bypassing the current measure resistor.
float GetPowerAmps();

// Returns the instantaneous power taken from the internal batteries or external power source, in watts.
// This is the power consumed by the robot's board, the motors, the servos and the Jetson Nano.
// If the robot carries a Jetson Orin Nano, its power is not included in the measure as it goes straight
// into the Jetson bypassing the current measure resistor.
float GetPowerWatts();

#endif  // POWER_INCLUDED_
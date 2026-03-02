#ifndef POWER_INCLUDED_
#define POWER_INCLUDED_

// Cuts power to the control board from the internal batteries or external power source.
// Calling this function is equivalent to pressing the power button until hardware shutdown. 
void PowerOff();

// Returns the instantaneous current taken from the internal batteries or external power source, in amps.
float GetCurrentAmps();

#endif  // POWER_INCLUDED_
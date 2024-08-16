/*
Motor control module.

Uses the channels in timer 1 to produce PWM control signals.
*/

void InitMotors();
void SetLeftMotorDutyCycle(float s);
void SetRightMotorDutyCycle(float s);

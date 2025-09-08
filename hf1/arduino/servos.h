/*
Servo control module.

Uses timer 0 channels to produce servo commands.
*/

void InitServos();

// Sets the yaw of the stereo pair in degrees.
// `angle_degrees` can vary between -90 and 90:
// 0: the head looks forward.
// -90: the head turns to the robot's right.
// 90: the head turns to the robot's right.
void SetHeadYawDegrees(float angle_degrees);

// Sets the pitch of the stereo pair in degrees.
// `angle_degrees` can vary between -90 and 90:
// 0: the robot eyes are looking forward.
// -90: the robot looks up at the sky.
// 90: the robot looks down at the floor.
void SetHeadPitchDegrees(float angle_degrees);

// Sets the roll of the stereo pair in degrees.
// `angle_degrees` can vary between -90 and 90:
// 0: the robot eyes are parallel to the floor.
// -90: the robot tilts its head clockwise, with its eyes perpendicular to the floor.
// 90: the robot tilts its head counterclockwise, with its eyes perpendicular to the floor.
void SetHeadRollDegrees(float angle_degrees);

// Saves the current servo angles as the origin.
void SaveServoAnglesAsOrigin();

float GetPitchServoMaxDegrees();
float GetPitchServoMinDegrees();
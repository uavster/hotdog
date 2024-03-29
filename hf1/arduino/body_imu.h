#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <status_or.h>

class BodyIMU {
public:
  BodyIMU();

  void Init();

  // The reference frame for all the following functions is as follows:
  // x points to the front of the robot.
  // y points to the left of the robot.
  // z points to the sky.

  // Returns the orientation with respect to the initial pose, in ZYX euler angles.
  // Value ranges are: yaw (z): [-pi, pi), pitch (y): [-pi/2, pi/2), roll (x): [-pi, pi).
  imu::Vector<3> GetYawPitchRoll();

  // Returns the linear accelerations in m/s^2.
  imu::Vector<3> GetLinearAccelerations();

  // Returns the angular velocities in rad/s.
  imu::Vector<3> GetAngularVelocities();

private:
  Adafruit_BNO055 bno_;
};
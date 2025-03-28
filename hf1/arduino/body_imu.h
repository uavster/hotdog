#include <status_or.h>
#include "bno055.h"
#include "vector.h"

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
  Vector<3> GetYawPitchRoll();

  // Returns the linear accelerations in m/s^2.
  Vector<3> GetLinearAccelerations();

  // Returns the angular velocities in rad/s.
  // imu::Vector<3> GetAngularVelocities();

  void StartCalibration();
  using CalibrationStatus = BNO055::CalibrationStatus;
  CalibrationStatus GetCalibrationStatus() const;
  using CalibrationData = BNO055::CalibrationData;
  CalibrationData GetCalibrationData() const;
  bool IsCalibrated() const;
  void StopCalibration();

private:
  BNO055 bno055_;
};

extern BodyIMU body_imu;
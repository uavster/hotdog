#include "Adafruit_BNO055.h"
#include "utility/vector.h"
#include <logger_interface.h>
#include "body_imu.h"
#include "timer.h"

#define kSensorID 55

BodyIMU::BodyIMU() : bno_(kSensorID) {}

void BodyIMU::Init() {
  ASSERT(bno_.begin(OPERATION_MODE_IMUPLUS));
}

imu::Vector<3> BodyIMU::GetYawPitchRoll() {
  // The IMU returns angles around x, y and z axes, where x points to the ground,
  // y points to the robot's right, and z points to the robot's back. 
  imu::Vector<3> euler = bno_.getVector(Adafruit_BNO055::VECTOR_EULER);
  // Transform to the canonical reference frame.
  // The yaw is in [0, 360), but we want it in [-180, 180).
  float yaw_symmetric = euler.x() <= 180 ? -euler.x() : 360 - euler.x();
  return imu::Vector<3>((-euler.z() * M_PI) / 180.0f, (-euler.y() * M_PI) / 180.0f, (yaw_symmetric * M_PI) / 180.0f);
}

imu::Vector<3> BodyIMU::GetLinearAccelerations() {
  return bno_.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

imu::Vector<3> BodyIMU::GetAngularVelocities() {
  imu::Vector<3> deg_s = bno_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  return imu::Vector<3>((deg_s.x() * M_PI) / 180.0f, (deg_s.y() * M_PI) / 180.0f, (deg_s.z() * M_PI) / 180.0f);
}

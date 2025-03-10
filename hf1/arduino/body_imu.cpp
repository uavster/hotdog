#include "Adafruit_BNO055.h"
#include "utility/vector.h"
#include "logger_interface.h"
#include "body_imu.h"
#include "timer.h"

#define kSensorID 55

#define kCalibrationMode false

BodyIMU::BodyIMU() : bno_(kSensorID) {}

void BodyIMU::Init() {
#if kCalibrationMode  
  ASSERT(bno_.begin(OPERATION_MODE_NDOF));
  uint8_t system = 0, gyro = 0, accel = 0, mag = 0;
  do {
    bno_.getCalibration(&system, &gyro, &accel, &mag);
    Serial.printf("Body IMU calibration status - system:%d gyro:%d accel:%d mag:%d\n", system, gyro, accel, mag);
    SleepForSeconds(0.5);
  } while(!bno_.isFullyCalibrated());
  Serial.printf("Body IMU calibration status - system:%d gyro:%d accel:%d mag:%d\n", system, gyro, accel, mag);  
  adafruit_bno055_offsets_t calibration_data;
  ASSERT(bno_.getSensorOffsets(calibration_data));
  for (size_t i = 0; i < sizeof(adafruit_bno055_offsets_t); ++i) {
    Serial.printf("0x%x", reinterpret_cast<uint8_t *>(&calibration_data)[i]);
    if (i < sizeof(adafruit_bno055_offsets_t) - 1) {
      Serial.printf(", ");
    }
  }
  while(true) {}
#else
  const static uint8_t calibration_data[] = { 
    0xe8, 0xff, 0xd8, 0xff, 0xd7, 0xff, 0xe, 0x1, 0x42, 0xf8, 0xbf, 
    0x0, 0x0, 0x0, 0xff, 0xff, 0x1, 0x0, 0xe8, 0x3, 0x33, 0x2 
  };
  ASSERT(bno_.begin(OPERATION_MODE_CONFIG));
  bno_.setSensorOffsets(calibration_data);
  bno_.setMode(OPERATION_MODE_IMUPLUS);
#endif
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

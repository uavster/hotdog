#include "logger_interface.h"
#include "body_imu.h"
#include "timer.h"

#define kSensorID 55

#define kCalibrationMode false

BodyIMU body_imu;

BodyIMU::BodyIMU() {}

void BodyIMU::Init() {
#if kCalibrationMode  
  ASSERT(bno055_.begin(BNO055_OPERATION_MODE_NDOF));
  uint8_t system = 0, gyro = 0, accel = 0, mag = 0;
  do {
    bno055_.getCalibration(&system, &gyro, &accel, &mag);
    Serial.printf("Body IMU calibration status - system:%d gyro:%d accel:%d mag:%d\n", system, gyro, accel, mag);
    SleepForSeconds(0.5);
  } while(!bno055_.isFullyCalibrated());
  Serial.printf("Body IMU calibration status - system:%d gyro:%d accel:%d mag:%d\n", system, gyro, accel, mag);  
  adafruit_bno055_offsets_t calibration_data;
  ASSERT(bno055_.getSensorOffsets(calibration_data));
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
  ASSERT(bno055_.begin(BNO055_OPERATION_MODE_CONFIG));
  bno055_.SetCalibrationData(*reinterpret_cast<const BNO055::CalibrationData *>(calibration_data));
  bno055_.setMode(BNO055_OPERATION_MODE_IMUPLUS);
#endif
}

Vector<3> BodyIMU::GetYawPitchRoll() {
  // The IMU returns angles around x, y and z axes, where x points to the ground,
  // y points to the robot's right, and z points to the robot's back. 
  const Vector euler = bno055_.getVector(TVectorType::VECTOR_EULER);
  // Transform to the canonical reference frame.
  // The yaw is in [0, 360), but we want it in [-180, 180).
  float yaw_symmetric = euler.x() <= 180 ? -euler.x() : 360 - euler.x();
  return Vector<3>((-euler.z() * M_PI) / 180.0f, (-euler.y() * M_PI) / 180.0f, (yaw_symmetric * M_PI) / 180.0f);
}

Vector<3> BodyIMU::GetLinearAccelerations() {
  return bno055_.getVector(TVectorType::VECTOR_LINEAR_ACCEL);
}

void BodyIMU::StartCalibration() {
  bno055_.begin(BNO055_OPERATION_MODE_NDOF);
}

bool BodyIMU::IsCalibrated() const {
  return GetCalibrationStatus().IsFullyCalibrated();
}

BodyIMU::CalibrationStatus BodyIMU::GetCalibrationStatus() const {
  return bno055_.GetCalibrationStatus();
}

BodyIMU::CalibrationData BodyIMU::GetCalibrationData() const {
  return bno055_.GetCalibrationData();
}

void BodyIMU::StopCalibration() {
  bno055_.setMode(BNO055_OPERATION_MODE_IMUPLUS);
}

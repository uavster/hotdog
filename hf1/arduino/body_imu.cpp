#include "logger_interface.h"
#include "body_imu.h"
#include "timer.h"
#include <EEPROM.h>

constexpr int kCalibrationDataEEPROMOffset = 0;
constexpr uint8_t kDeviceAddress = 0x28;

BodyIMU body_imu;

BodyIMU::BodyIMU() : bno055_(kDeviceAddress) {}

void BodyIMU::Init() {
  ASSERT(bno055_.begin(BNO055_OPERATION_MODE_CONFIG));
  if (LoadCalibrationData()) {
    LOG_INFO("Body IMU calibration data loaded from EEPROM.");
  } else {
    LOG_WARNING("Unable to load body IMU calibration data. Please recalibrate!");
    // Fallback calibration in case EEPROM copy fails. Causes: faulty EEPROM, corrupted data, new CalibrationData size.
    const static uint8_t calibration_data[] = { 
      0xe8, 0xff, 0xd8, 0xff, 0xd7, 0xff, 0xe, 0x1, 0x42, 0xf8, 0xbf, 
      0x0, 0x0, 0x0, 0xff, 0xff, 0x1, 0x0, 0xe8, 0x3, 0x33, 0x2 
    };
    bno055_.SetCalibrationData(*reinterpret_cast<const BNO055::CalibrationData *>(calibration_data));
  }
  bno055_.setMode(BNO055_OPERATION_MODE_IMUPLUS);
}

void BodyIMU::Run() {
  bno055_.Run();
}

Vector<3> BodyIMU::GetYawPitchRoll() {
  return CanonicalizeEulerVector(bno055_.getVector(TVectorType::VECTOR_EULER));
}

Vector<3> BodyIMU::GetRawAccelerations() {
  return bno055_.getVector(TVectorType::VECTOR_RAW_ACCEL);
}

Vector<3> BodyIMU::GetLinearAccelerations() {
  return bno055_.getVector(TVectorType::VECTOR_LINEAR_ACCEL);
}

void BodyIMU::StartCalibration() {
  bno055_.begin(BNO055_OPERATION_MODE_NDOF);
}

bool BodyIMU::IsCalibrated() {
  return GetCalibrationStatus().IsFullyCalibrated();
}

BodyIMU::CalibrationStatus BodyIMU::GetCalibrationStatus() {
  return bno055_.GetCalibrationStatus();
}

BodyIMU::CalibrationData BodyIMU::GetCalibrationData() {
  return bno055_.GetCalibrationData();
}

void BodyIMU::StopCalibration() {
  bno055_.setMode(BNO055_OPERATION_MODE_IMUPLUS);
}
#include <Arduino.h>

bool BodyIMU::LoadCalibrationData() {
  uint8_t calibration_data_size = 0;
  if (EEPROM.get(kCalibrationDataEEPROMOffset, calibration_data_size) != sizeof(CalibrationData)) {
    return false;
  }
  CalibrationData calibration_data;
  EEPROM.get(kCalibrationDataEEPROMOffset + sizeof(kCalibrationDataEEPROMOffset), calibration_data);
  bno055_.SetCalibrationData(calibration_data);
  return true;
}

bool BodyIMU::SaveCalibrationData() {
  EEPROM.update(kCalibrationDataEEPROMOffset, static_cast<uint8_t>(sizeof(CalibrationData)));
  EEPROM.put(kCalibrationDataEEPROMOffset + sizeof(kCalibrationDataEEPROMOffset), bno055_.GetCalibrationData());
  return true;
}

Status BodyIMU::AsyncRequestYawPitchRoll() {
  return bno055_.RequestVectorAsync(TVectorType::VECTOR_EULER);
}

Status BodyIMU::AsyncRequestLinearAccelerations() {
  return bno055_.RequestVectorAsync(TVectorType::VECTOR_LINEAR_ACCEL);
}

Status BodyIMU::AsyncRequestRawAccelerations() {
  return bno055_.RequestVectorAsync(TVectorType::VECTOR_RAW_ACCEL);
}

StatusOr<Vector<3>> BodyIMU::GetLastYawPitchRoll() {
  StatusOr<Vector<3>> maybe_vector = bno055_.GetLastRequestedVector(TVectorType::VECTOR_EULER);
  if (!maybe_vector.ok()) {
    return maybe_vector;
  }
  return CanonicalizeEulerVector(*maybe_vector);
}

StatusOr<Vector<3>> BodyIMU::GetLastLinearAccelerations() {
  return bno055_.GetLastRequestedVector(TVectorType::VECTOR_LINEAR_ACCEL);
}

StatusOr<Vector<3>> BodyIMU::GetLastRawAccelerations() {
  return bno055_.GetLastRequestedVector(TVectorType::VECTOR_RAW_ACCEL);
}

Vector<3> BodyIMU::CanonicalizeEulerVector(const Vector<3> &vector) {
  // The IMU returns angles around x, y and z axes, where x points to the ground,
  // y points to the robot's right, and z points to the robot's back. 
  // Transform to the canonical reference frame.
  // The yaw is in [0, 360), but we want it in [-180, 180).
  float yaw_symmetric = vector.x() <= 180 ? -vector.x() : 360 - vector.x();
  return Vector<3>((-vector.z() * M_PI) / 180.0f, (-vector.y() * M_PI) / 180.0f, (yaw_symmetric * M_PI) / 180.0f);
}

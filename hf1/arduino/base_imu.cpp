#include "logger_interface.h"
#include "base_imu.h"
#include "timer.h"
#include <EEPROM.h>

constexpr uint8_t kDeviceAddress = 0x28;

BaseIMU::BaseIMU(int calibration_data_offset) : bno055_(kDeviceAddress), calibration_data_offset_(calibration_data_offset) {}

void BaseIMU::Init() {
  ASSERT(bno055_.begin(BNO055_OPERATION_MODE_CONFIG));
  if (LoadCalibrationData()) {
    LOG_INFO("Base IMU calibration data loaded from EEPROM.");
  } else {
    LOG_WARNING("Unable to load base IMU calibration data. Please recalibrate!");
    // Fallback calibration in case EEPROM copy fails. Causes: faulty EEPROM, corrupted data, new CalibrationData size.
    const static uint8_t calibration_data[] = { 
      0xe8, 0xff, 0xd8, 0xff, 0xd7, 0xff, 0xe, 0x1, 0x42, 0xf8, 0xbf, 
      0x0, 0x0, 0x0, 0xff, 0xff, 0x1, 0x0, 0xe8, 0x3, 0x33, 0x2 
    };
    bno055_.SetCalibrationData(*reinterpret_cast<const BNO055::CalibrationData *>(calibration_data));
  }
  bno055_.setMode(BNO055_OPERATION_MODE_IMUPLUS);
}

void BaseIMU::Run() {
  bno055_.Run();
}

Vector<3> BaseIMU::GetYawPitchRoll() {
  return CanonicalizeEulerVector(bno055_.getVector(TVectorType::VECTOR_EULER));
}

Vector<3> BaseIMU::GetRawAccelerations() {
  return bno055_.getVector(TVectorType::VECTOR_RAW_ACCEL);
}

Vector<3> BaseIMU::GetLinearAccelerations() {
  return bno055_.getVector(TVectorType::VECTOR_LINEAR_ACCEL);
}

void BaseIMU::StartCalibration() {
  bno055_.begin(BNO055_OPERATION_MODE_NDOF);
}

bool BaseIMU::IsCalibrated() {
  return GetCalibrationStatus().IsFullyCalibrated();
}

BaseIMU::CalibrationStatus BaseIMU::GetCalibrationStatus() {
  return bno055_.GetCalibrationStatus();
}

BaseIMU::CalibrationData BaseIMU::GetCalibrationData() {
  return bno055_.GetCalibrationData();
}

void BaseIMU::StopCalibration() {
  bno055_.setMode(BNO055_OPERATION_MODE_IMUPLUS);
}

bool BaseIMU::LoadCalibrationData() {
  uint8_t calibration_data_size = 0;
  if (EEPROM.get(calibration_data_offset_, calibration_data_size) != sizeof(CalibrationData)) {
    return false;
  }
  CalibrationData calibration_data;
  EEPROM.get(calibration_data_offset_ + 1, calibration_data);
  bno055_.SetCalibrationData(calibration_data);
  return true;
}

bool BaseIMU::SaveCalibrationData() {
  EEPROM.update(calibration_data_offset_, static_cast<uint8_t>(sizeof(CalibrationData)));
  EEPROM.put(calibration_data_offset_ + 1, bno055_.GetCalibrationData());
  return true;
}

Status BaseIMU::AsyncRequestYawPitchRoll() {
  return bno055_.RequestVectorAsync(TVectorType::VECTOR_EULER);
}

Status BaseIMU::AsyncRequestLinearAccelerations() {
  return bno055_.RequestVectorAsync(TVectorType::VECTOR_LINEAR_ACCEL);
}

Status BaseIMU::AsyncRequestRawAccelerations() {
  return bno055_.RequestVectorAsync(TVectorType::VECTOR_RAW_ACCEL);
}

StatusOr<Vector<3>> BaseIMU::GetLastYawPitchRoll() {
  StatusOr<Vector<3>> maybe_vector = bno055_.GetLastRequestedVector(TVectorType::VECTOR_EULER);
  if (!maybe_vector.ok()) {
    return maybe_vector;
  }
  return CanonicalizeEulerVector(*maybe_vector);
}

StatusOr<Vector<3>> BaseIMU::GetLastLinearAccelerations() {
  return bno055_.GetLastRequestedVector(TVectorType::VECTOR_LINEAR_ACCEL);
}

StatusOr<Vector<3>> BaseIMU::GetLastRawAccelerations() {
  return bno055_.GetLastRequestedVector(TVectorType::VECTOR_RAW_ACCEL);
}

Vector<3> BaseIMU::CanonicalizeEulerVector(const Vector<3> &vector) {
  // The IMU returns angles around x, y and z axes, where x points to the ground,
  // y points to the robot's right, and z points to the robot's back. 
  // Transform to the canonical reference frame.
  // The yaw is in [0, 360), but we want it in [-180, 180).
  float yaw_symmetric = vector.x() <= 180 ? -vector.x() : 360 - vector.x();
  return Vector<3>((-vector.z() * M_PI) / 180.0f, (-vector.y() * M_PI) / 180.0f, (yaw_symmetric * M_PI) / 180.0f);
}

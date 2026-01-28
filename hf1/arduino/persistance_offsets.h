#include "base_imu.h"

constexpr int kPersistanceOffsetBaseIMUCalibration = 0;
constexpr int kPersistanceOffsetServoCalibration = kPersistanceOffsetBaseIMUCalibration + BaseIMU::kCalibrationDataSizeOnStorage;
#include "base_imu.h"

constexpr int kEEPROMOffsetBaseIMUCalibration = 0;
constexpr int kEEPROMOffsetServoCalibration = kEEPROMOffsetBaseIMUCalibration + BaseIMU::kCalibrationDataSizeOnEEPROM;
#ifndef BNO055_INCLUDED_
#define BNO055_INCLUDED_

#include <stdint.h>
#include "vector.h"

// Operation mode settings.
typedef enum {
  BNO055_OPERATION_MODE_CONFIG = 0X00,
  BNO055_OPERATION_MODE_ACCONLY = 0X01,
  BNO055_OPERATION_MODE_MAGONLY = 0X02,
  BNO055_OPERATION_MODE_GYRONLY = 0X03,
  BNO055_OPERATION_MODE_ACCMAG = 0X04,
  BNO055_OPERATION_MODE_ACCGYRO = 0X05,
  BNO055_OPERATION_MODE_MAGGYRO = 0X06,
  BNO055_OPERATION_MODE_AMG = 0X07,
  BNO055_OPERATION_MODE_IMUPLUS = 0X08,
  BNO055_OPERATION_MODE_COMPASS = 0X09,
  BNO055_BNO055_OPERATION_MODE_M4G = 0X0A,
  BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
  BNO055_OPERATION_MODE_NDOF = 0X0C
} BNO055OperationMode;

typedef enum {
  VECTOR_EULER = 0x1a,
  VECTOR_LINEAR_ACCEL = 0x28
} TVectorType;

// Class handling communication with the BNO055 IMU.
class BNO055 {
public:
  BNO055() {}
  bool begin(BNO055OperationMode op_mode);
  void setMode(BNO055OperationMode op_mode);
  Vector<3> getVector(TVectorType vector_type) const;

#pragma pack(push, 1)
  struct CalibrationStatus {
    uint8_t system;
    uint8_t gyroscopes;
    uint8_t accelerometers;
    uint8_t magnetometer;

    bool IsSystemCalibrated() const { return system == 3; }
    bool AreGyroscopesCalibrated() const { return gyroscopes == 3; }
    bool AreAccelerometersCalibrated() const { return accelerometers == 3; }
    bool IsMagnetometerCalibrated() const { return magnetometer == 3; }
    bool IsFullyCalibrated() const {
      return IsSystemCalibrated() && AreGyroscopesCalibrated() && AreAccelerometersCalibrated() && IsMagnetometerCalibrated();
    }
  };
  CalibrationStatus GetCalibrationStatus() const;

  struct CalibrationData {
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;

    int16_t mag_offset_x;
    int16_t mag_offset_y;
    int16_t mag_offset_z;

    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;

    int16_t accel_radius;

    int16_t mag_radius;
  };
#pragma pack(pop)
  CalibrationData GetCalibrationData() const;
  void SetCalibrationData(const CalibrationData &data) const;

private:
  BNO055OperationMode last_mode_;
};

#endif  // BNO055_INCLUDED_
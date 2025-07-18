#ifndef BASE_IMU_INCLUDED_
#define BASE_IMU_INCLUDED_

#include <status_or.h>
#include "bno055.h"
#include "vector.h"
#include "status_or.h"

class BaseIMU {
public:
  // `calibration_data_offset` indicates where in the EEPROM the calibration data should be placed
  // by SaveCalibrationData().
  explicit BaseIMU(int calibration_data_offset);

  void Init();
  void Run();

  // The reference frame for all the following functions is as follows:
  // x points to the front of the robot.
  // y points to the left of the robot.
  // z points to the sky.

  // Returns the orientation with respect to the initial pose, in ZYX euler angles.
  // Value ranges are: yaw (z): [-pi, pi), pitch (y): [-pi/2, pi/2), roll (x): [-pi, pi).
  // Unlike its AsyncRequest*() counterpart, this function blocks until the result is available.
  Vector<3> GetYawPitchRoll();

  // Returns the linear accelerations in m/s^2.
  // The function blocks until the result is available.
  // Unlike its AsyncRequest*() counterpart, this function blocks until the result is available.
  Vector<3> GetLinearAccelerations();

  // Returns the raw accelerations in m/s^2.
  // Unlike its AsyncRequest*() counterpart, this function blocks until the result is available.
  Vector<3> GetRawAccelerations();

  // These functions are like their Get*() counterparts, but retrieve the result asynchronously.
  // To get the result when available call their respective GetLast*() functions.
  //  kSuccess - the vector was requested correctly.
  //  kUnavailablError - there was an error requesting the vector.
  //  kExistsError - there already exists an asynchronous operation in progress.
  Status AsyncRequestYawPitchRoll();
  Status AsyncRequestLinearAccelerations();
  Status AsyncRequestRawAccelerations();

  // These functions get the vectors requested with the last call to the respective AsyncRequest*(), if available.
  // They may return the following status codes:
  //  kInProgressError - the vector is being retrieved.
  //  kSuccess - the vector was received and is returned.
  //  kUnavailablError - there was an error retrieving the vector.
  //  kDoesNotExistError - no asynchronous operation was started with AsyncRequest*() to retrieve the vector.
  StatusOr<Vector<3>> GetLastYawPitchRoll();
  StatusOr<Vector<3>> GetLastLinearAccelerations();
  StatusOr<Vector<3>> GetLastRawAccelerations();

  // All the following functions wait until the pending asynchronous request is complete, if any.
  void StartCalibration();
  using CalibrationStatus = BNO055::CalibrationStatus;
  CalibrationStatus GetCalibrationStatus();
  using CalibrationData = BNO055::CalibrationData;

  static constexpr int kCalibrationDataSizeOnEEPROM = sizeof(CalibrationData) + 1;

  CalibrationData GetCalibrationData();
  bool SaveCalibrationData();
  bool LoadCalibrationData();
  bool IsCalibrated();
  void StopCalibration();

private:
  BNO055 bno055_;
  Vector<3> CanonicalizeEulerVector(const Vector<3> &vector);
  int calibration_data_offset_;
};

extern BaseIMU base_imu;

#endif  // BASE_IMU_INCLUDED_
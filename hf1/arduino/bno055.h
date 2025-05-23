#ifndef BNO055_INCLUDED_
#define BNO055_INCLUDED_

#include <stdint.h>
#include "vector.h"
#include "status_or.h"

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
  BNO055_OPERATION_MODE_M4G = 0X0A,
  BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
  BNO055_OPERATION_MODE_NDOF = 0X0C
} BNO055OperationMode;

typedef enum {
  VECTOR_RAW_ACCEL = 0x8,
  VECTOR_EULER = 0x1a,
  VECTOR_LINEAR_ACCEL = 0x28
} TVectorType;

// Class handling communication with the BNO055 IMU.
class BNO055 {
public:
  BNO055(uint8_t device_address);

  // Initializes the IMU or dies if there's any error.
  // Before reinitializing it, it wait until any pending asynchronous transfers finish.
  bool begin(BNO055OperationMode op_mode);

  // Sets the operating mode of the IMU or dies if there's any error.
  // Before changing the mode, it waits until any pending asynchronous transfers finish.
  void setMode(BNO055OperationMode op_mode);

  // Blocks until the requested vector type is received and returns it, or dies if there's any error.
  // If there is a pending asynchronous transfer initiated with RequestVectorAsync(), this function waits for it to finish,
  // and initiates a new one, if the requested vector type is different.
  // The call dies upon transmission errors.
  Vector<3> getVector(TVectorType vector_type);

  // Initiates an asynchronous transfer of the passed vector type.
  // This function returns immediately. Callers can call GetLastRequestedVector() to wait for the requested data.
  // Before the requested data arrives, further calls to RequestVectorAsync() will fail.
  // This function may return one of the following status codes: 
  //  kSuccess - the transfer was initiated correctly.
  //  kExistsError - there already exists an asynchronous operation in progress.
  //  kUnavailableError - there was a communication error.
  Status RequestVectorAsync(TVectorType vector_type);

  // Gets the last vector of the given type requested with RequestVectorAsync(), if available. 
  // This function may return one of the following status codes:
  //  kSuccess - the vector is available and returned.
  //  kUnavailableError - there was a communication error.
  //  kInProgressError - the vector is being requested or transferred.
  //  kDoesNotExistError - no asynchronous operation was started with RequestVectorAsync() for the given type.
  StatusOr<Vector<3>> GetLastRequestedVector(TVectorType vector_type) const;

  // Returns the vector type of the transfer in progress, if any, or kDoesNotExistError if no transfer is in progress.
  StatusOr<TVectorType> GetVectorTypeOfTransferInProgress() const;

#pragma pack(push, 1)
  struct CalibrationStatus {
    BNO055OperationMode operation_mode;
    uint8_t system;
    uint8_t gyroscopes;
    uint8_t accelerometers;
    uint8_t magnetometer;

    bool IsSystemCalibrated() const { return system == 3; }
    bool AreGyroscopesCalibrated() const { return gyroscopes == 3; }
    bool AreAccelerometersCalibrated() const { return accelerometers == 3; }
    bool IsMagnetometerCalibrated() const { return magnetometer == 3; }

    bool IsFullyCalibrated() const;
  };

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

  // Returns the calibration status of the IMU.
  // Before requesting the calibration status, it waits until any pending asynchronous transfers finish.
  CalibrationStatus GetCalibrationStatus();

  // Returns the calibration data or dies if there's any error.
  // Before requesting the calibration data, it waits until any pending asynchronous transfers finish.
  CalibrationData GetCalibrationData();

  // Sets the calibration data or dies if there's any error.
  // Before setting the calibration data, it waits until any pending asynchronous transfers finish.
  void SetCalibrationData(const CalibrationData &data);

  // Handles the lifetime of asynchronous requests. Must be called periodically.
  void Run();

  // Blocks until the pending transfer finishes, if any.
  void WaitForPendingTransferToFinish();

private:
  int GetLastVectorIndexFromVectorType(TVectorType type) const;
  TVectorType GetVectorTypeFromLastVectorIndex(int index) const;

  uint8_t device_address_;
  BNO055OperationMode last_mode_;
  StatusOr<Vector<3>> last_vectors_[3];  

  using State = enum { kIdle, kWaitingForRegisterAddressWritten, kReadingData };
  State state_;
};

#endif  // BNO055_INCLUDED_
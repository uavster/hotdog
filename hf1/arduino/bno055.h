#include <stdint.h>

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

class Vector {
public:
  Vector() : x_(0), y_(0), z_(0) {}
  Vector(float x, float y, float z) : x_(x), y_(y), z_(z) {}

  float x() const { return x_; }
  float y() const { return y_; }
  float z() const { return z_; }

private:
  float x_;
  float y_;
  float z_;
};

// Class handling communication with the BNO055 IMU.
class BNO055 {
public:
  BNO055() {}
  bool begin(BNO055OperationMode op_mode);
  void setMode(BNO055OperationMode op_mode);
  void setSensorOffsets(const uint8_t *calibration_data);
  Vector getVector(TVectorType vector_type);

private:
  BNO055OperationMode last_mode_;
};
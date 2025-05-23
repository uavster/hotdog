#include "core_pins.h"
#include "bno055.h"
#include "i2c_t3.h"
#include "timer.h"
#include "logger_interface.h"

#define EXPECTED_CHIP_ID  0xa0

#define CHIP_ID_REG             0

#define PAGE_ID_REG             0x7

#define OPERATION_MODE_POS			0
#define OPERATION_MODE_MASK			0xf
#define OPERATION_MODE_REG			0x3d
#define SYS_TRIGGER_REG         0x3f
#define SYS_CLK_STATUS_REG      0x38
#define CALIBRATION_STATUS_REG  0x35

#define SENSOR_OFFSETS_BASE_ADDRESS   0x55
#define SENSOR_OFFSETS_NUM_REGISTERS  22

constexpr TimerNanosType kI2CBusFrequency = 100'000;
constexpr TimerNanosType kReadTimeoutPerByteNs = (20 * 1'000'000'000ULL) / kI2CBusFrequency;
constexpr TimerNanosType kReadTimeoutConstantNs = 100'000'000;

static void ReadFromI2C(uint8_t device_address, uint8_t register_address, void *data, uint8_t length) {
	Wire.beginTransmission(device_address);	
	Wire.write(register_address);
	const auto tx_result = Wire.endTransmission();
  switch(tx_result) {
    case 0: break;  // Success.
    case 1: ASSERTM(false, "ReadFromI2C ERROR: data too long.");
    case 2: ASSERTM(false, "ReadFromI2C ERROR: no ACK to address.");
    case 3: ASSERTM(false, "ReadFromI2C ERROR: no ACK to data.");
    default: ASSERTM(false, "ReadFromI2C ERROR: unknown error.");
  }
  SleepForNanos(150000);
	Wire.requestFrom(device_address, length);
  auto bytes = reinterpret_cast<uint8_t *>(data);
  const auto start_ns = GetTimerNanoseconds();
  const TimerNanosType timeout_ns = length * kReadTimeoutPerByteNs + kReadTimeoutConstantNs;
	while(Wire.available() && GetTimerNanoseconds() - start_ns < timeout_ns) {
		*(bytes++) = Wire.read();
    --length;
	}
  ASSERTM(length == 0, "ReadFromI2C ERROR: device responded by the timeout expired reading data.");
}

static void WriteToI2C(uint8_t device_address, uint8_t register_address, const void *data, uint8_t length) {
	Wire.beginTransmission(device_address);
	Wire.write(register_address);	
  auto bytes = reinterpret_cast<const uint8_t *>(data);
	for(uint8_t i = 0; i < length; ++i) {
		Wire.write(*bytes);
		++bytes;
	}
	Wire.endTransmission();
	SleepForNanos(150000);
}

static uint8_t ReadByteFromI2C(uint8_t device_address, uint8_t reg_addr) {
  uint8_t data;
  ReadFromI2C(device_address, reg_addr, &data, 1);
  return data;
}

static void WriteByteToI2C(uint8_t device_address, uint8_t reg_addr, uint8_t value) {
  WriteToI2C(device_address, reg_addr, &value, 1);
}

static uint8_t SetBytePart(uint8_t dest, uint8_t offset, uint8_t mask, uint8_t value) {
  return (dest & (~mask)) | ((value & mask) << offset);
}

static uint8_t GetBytePart(uint8_t source, uint8_t offset, uint8_t mask) {
  return (source & mask) >> offset;
}

static void WritePageID(uint8_t device_address, uint8_t page_id) {
  WriteByteToI2C(device_address, PAGE_ID_REG, page_id); 
}

static BNO055OperationMode GetOperationMode(uint8_t device_address) {
  WritePageID(device_address, 0);
  return static_cast<BNO055OperationMode>(GetBytePart(ReadByteFromI2C(device_address, OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK));
}

static void SetOperationMode(uint8_t device_address, BNO055OperationMode op_mode) {
  if (GetOperationMode(device_address) == BNO055_OPERATION_MODE_CONFIG) {
    WriteByteToI2C(device_address, 
      OPERATION_MODE_REG, 
      SetBytePart(ReadByteFromI2C(device_address, OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK, op_mode)
    );
  } else {
    WriteByteToI2C(device_address, 
      OPERATION_MODE_REG, 
      SetBytePart(ReadByteFromI2C(device_address, OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK, BNO055_OPERATION_MODE_CONFIG) 
    );
    if (op_mode != BNO055_OPERATION_MODE_CONFIG) {
      WriteByteToI2C(device_address,
        OPERATION_MODE_REG, 
        SetBytePart(ReadByteFromI2C(device_address, OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK, op_mode)
      );
    }
  }
}

BNO055::BNO055(uint8_t device_address) : device_address_(device_address), state_(State::kIdle) {
  for (size_t i = 0; i < sizeof(last_vectors_) / sizeof(last_vectors_[0]); ++i) {
    // Mark this type of vector as "not yet requested".
    last_vectors_[i] = Status::kDoesNotExistError;
  }
}

bool BNO055::begin(BNO055OperationMode op_mode) {
  WaitForPendingTransferToFinish();

  // Hard reset.
  pinMode(27, OUTPUT);
  digitalWrite(27, 0);
  SleepForSeconds(0.01);
  pinMode(27, INPUT);
  SleepForNanos(650'000'000);  // Typical POR time is 650 ms.

  Wire.begin();
  Wire.setDefaultTimeout(100'000/* us */);
  SleepForNanos(1'000'000);
  
  // Wait for the chip to start up and check that we have the right model.
  SleepForNanos(400'000'000); // Typical start-up time is 400 ms.
  int timeout_ms = 200;  
  while(ReadByteFromI2C(device_address_, CHIP_ID_REG) != EXPECTED_CHIP_ID && timeout_ms > 0) {
    SleepForNanos(10'000'000);
    timeout_ms -= 10;
  }
  ASSERTM(timeout_ms > 0, "IMU does not start or it's not a BNO055.");

  // Reset the chip in case begin() has been called a second time, 
  // or the MCU was reset by the programmer without power-cycling the IMU.
  SetOperationMode(device_address_, BNO055_OPERATION_MODE_CONFIG);
  WriteByteToI2C(device_address_, SYS_TRIGGER_REG, 0x20);

  // Check that we have the right chip. Retry until it is on.
  SleepForNanos(650'000'000);  // Typical POR time is 650 ms.
  timeout_ms = 200;
  while(ReadByteFromI2C(device_address_, CHIP_ID_REG) != EXPECTED_CHIP_ID && timeout_ms > 0) {
    SleepForNanos(10'000'000);
    timeout_ms -= 10;
  }
  ASSERTM(timeout_ms > 0, "IMU does not start or it's not a BNO055.");
  SleepForNanos(50'000'000);

  // Configure external crystal.
  SetOperationMode(device_address_, BNO055_OPERATION_MODE_CONFIG);
  SleepForNanos(25'000'000);
  WritePageID(device_address_, 0);
  WriteByteToI2C(device_address_, SYS_TRIGGER_REG, 0x80);
  SleepForNanos(600'000'000);
  timeout_ms = 1000;
  while((ReadByteFromI2C(device_address_, SYS_CLK_STATUS_REG) & 1) && timeout_ms > 0) {
    SleepForNanos(10'000'000);
    timeout_ms -= 10;
  }
  ASSERTM(!(ReadByteFromI2C(device_address_, SYS_CLK_STATUS_REG) & 1), "Timed out waiting for ST_MAIN_CLK to become low.");
  ASSERTM(ReadByteFromI2C(device_address_, SYS_TRIGGER_REG) & 0x80, "Unable to start external oscillator.");

  setMode(op_mode);
  SleepForNanos(20'000'000);

  return true;
}

void BNO055::setMode(BNO055OperationMode op_mode) {
  WaitForPendingTransferToFinish();
  SetOperationMode(device_address_, op_mode);
  last_mode_ = op_mode;
}

void BNO055::WaitForPendingTransferToFinish() {
  const StatusOr<TVectorType> in_progress_type = GetVectorTypeOfTransferInProgress();
  if (!in_progress_type.ok()) { return; }
  while(GetLastRequestedVector(*in_progress_type).status() == Status::kInProgressError) {
    Run();  
  }
}

Vector<3> BNO055::getVector(TVectorType vector_type) {
  // If there is a pending asynchronous transfer, wait for it to finish.
  const StatusOr<TVectorType> in_progress_type = GetVectorTypeOfTransferInProgress();
  if (in_progress_type.ok()) {
    WaitForPendingTransferToFinish();
    // Return the vector if it matches the requested type and was received correctly.
    if (*in_progress_type == vector_type && GetLastRequestedVector(*in_progress_type).ok()) {
      return *GetLastRequestedVector(*in_progress_type);
    }
  }

  // Assume that page 0 is selected.
  uint8_t buffer[6];
  ReadFromI2C(device_address_, static_cast<uint8_t>(vector_type), buffer, sizeof(buffer) / sizeof(buffer[0]));
  const int16_t x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  const int16_t y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  const int16_t z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  switch(vector_type) {
    case VECTOR_RAW_ACCEL:
      return Vector<3>(static_cast<float>(x) / 100.0, static_cast<float>(y) / 100.0, static_cast<float>(z) / 100.0);
    case VECTOR_EULER:
      return Vector<3>(static_cast<float>(x) / 16.0, static_cast<float>(y) / 16.0, static_cast<float>(z) / 16.0);
    case VECTOR_LINEAR_ACCEL:
      return Vector<3>(static_cast<float>(x) / 100.0, static_cast<float>(y) / 100.0, static_cast<float>(z) / 100.0);
  }
  return Vector<3>();
}

BNO055::CalibrationStatus BNO055::GetCalibrationStatus() {
  WaitForPendingTransferToFinish();
  const uint8_t reg = ReadByteFromI2C(device_address_, CALIBRATION_STATUS_REG);  
  return {
    .operation_mode = GetOperationMode(device_address_),
    .system = static_cast<uint8_t>((reg >> 6) & 0b11),
    .gyroscopes = static_cast<uint8_t>((reg >> 4) & 0b11),
    .accelerometers = static_cast<uint8_t>((reg >> 2) & 0b11),
    .magnetometer = static_cast<uint8_t>(reg & 0b11)
  };
}

BNO055::CalibrationData BNO055::GetCalibrationData() {
  WaitForPendingTransferToFinish();
  SetOperationMode(device_address_, BNO055_OPERATION_MODE_CONFIG);
  CalibrationData calibration_data;
  ReadFromI2C(device_address_, SENSOR_OFFSETS_BASE_ADDRESS, &calibration_data, SENSOR_OFFSETS_NUM_REGISTERS);
  SetOperationMode(device_address_, last_mode_);
  return calibration_data;
}

bool BNO055::CalibrationStatus::IsFullyCalibrated() const {
  switch (operation_mode) {
    case BNO055_OPERATION_MODE_ACCONLY:
      return AreAccelerometersCalibrated();
    case BNO055_OPERATION_MODE_MAGONLY:
      return IsMagnetometerCalibrated();
    case BNO055_OPERATION_MODE_GYRONLY:
    case BNO055_OPERATION_MODE_M4G: /* No magnetometer calibration required. */
      return AreGyroscopesCalibrated();
    case BNO055_OPERATION_MODE_ACCMAG:
    case BNO055_OPERATION_MODE_COMPASS:
      return AreAccelerometersCalibrated() && IsMagnetometerCalibrated();
    case BNO055_OPERATION_MODE_ACCGYRO:
    case BNO055_OPERATION_MODE_IMUPLUS:
      return AreAccelerometersCalibrated() && AreGyroscopesCalibrated();
    case BNO055_OPERATION_MODE_MAGGYRO:
      return IsMagnetometerCalibrated() && AreGyroscopesCalibrated();
    default:
      return IsSystemCalibrated() && AreAccelerometersCalibrated() && AreGyroscopesCalibrated() && IsMagnetometerCalibrated();
  }  
}

void BNO055::SetCalibrationData(const CalibrationData &data) {
  WaitForPendingTransferToFinish();
  SetOperationMode(device_address_, BNO055_OPERATION_MODE_CONFIG);
  SleepForNanos(25000000);
  WriteToI2C(device_address_, SENSOR_OFFSETS_BASE_ADDRESS, &data, SENSOR_OFFSETS_NUM_REGISTERS);
  SetOperationMode(device_address_, last_mode_);
}

int BNO055::GetLastVectorIndexFromVectorType(TVectorType type) const {
  switch(type) {
    case VECTOR_EULER: return 0;
    case VECTOR_LINEAR_ACCEL: return 1;
    case VECTOR_RAW_ACCEL: return 2;
    default: { ASSERT(false); return 0; }
  }
}

TVectorType BNO055::GetVectorTypeFromLastVectorIndex(int index) const {
  const static TVectorType vector_type_from_index[] = { VECTOR_EULER, VECTOR_LINEAR_ACCEL, VECTOR_RAW_ACCEL };
  ASSERT(index >= 0);
  ASSERT(index < static_cast<int>(sizeof(vector_type_from_index) / sizeof(vector_type_from_index[0])));
  return vector_type_from_index[index];
}

StatusOr<TVectorType> BNO055::GetVectorTypeOfTransferInProgress() const {
  for (size_t i = 0; i < sizeof(last_vectors_) / sizeof(last_vectors_[0]); ++i) {
    const auto &last_vector = last_vectors_[i];
    if (last_vector.status() == Status::kInProgressError) {
      return GetVectorTypeFromLastVectorIndex(i);
    } 
  }
  return Status::kDoesNotExistError;
}

Status BNO055::RequestVectorAsync(TVectorType vector_type) {
  // Fail if there is any request in progress.
  if (GetVectorTypeOfTransferInProgress().ok()) {
    return Status::kExistsError;
  }

  // Mark the vector type as being requested.
  last_vectors_[GetLastVectorIndexFromVectorType(vector_type)] = Status::kInProgressError;

  // Initiate tx request to send register address.
	Wire.beginTransmission(device_address_);	
	ASSERT(Wire.write(static_cast<uint8_t>(vector_type)) == 1);
  Wire.sendTransmission();
  
  state_ = State::kWaitingForRegisterAddressWritten;
  return Status::kSuccess;
}

StatusOr<Vector<3>> BNO055::GetLastRequestedVector(TVectorType vector_type) const {
  return last_vectors_[GetLastVectorIndexFromVectorType(vector_type)];
}

void BNO055::Run() {
  switch(state_) {
    case State::kIdle: break;

    case State::kWaitingForRegisterAddressWritten:
      if (!Wire.done()) { break; }
      if (Wire.getError() != 0) {
        const auto in_progress_type = GetVectorTypeOfTransferInProgress();
        ASSERT(in_progress_type.ok());
        last_vectors_[GetLastVectorIndexFromVectorType(*in_progress_type)] = Status::kUnavailableError;
        state_ = kIdle;
        break;
      }

      Wire.sendRequest(device_address_, sizeof(Vector<3>));
      state_ = State::kReadingData;
      break;

    case State::kReadingData:
      if (Wire.done()) { break; }
      state_ = State::kIdle;
      const auto in_progress_type = GetVectorTypeOfTransferInProgress();
      ASSERT(in_progress_type.ok());
      auto &last_vector = last_vectors_[GetLastVectorIndexFromVectorType(*in_progress_type)];
      if (Wire.getError() != 0) {
        last_vector = Status::kUnavailableError;
        break;
      }

      uint8_t buffer[6];
      ASSERT(Wire.readBytes(buffer, sizeof(buffer) / sizeof(buffer[0])) == sizeof(buffer) / sizeof(buffer[0]));
      const int16_t x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
      const int16_t y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
      const int16_t z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

      switch(*in_progress_type) {
        case VECTOR_RAW_ACCEL:
          last_vector = Vector<3>(static_cast<float>(x) / 100.0, static_cast<float>(y) / 100.0, static_cast<float>(z) / 100.0);
          break;
        case VECTOR_EULER:
          last_vector = Vector<3>(static_cast<float>(x) / 16.0, static_cast<float>(y) / 16.0, static_cast<float>(z) / 16.0);
          break;
        case VECTOR_LINEAR_ACCEL:
          last_vector = Vector<3>(static_cast<float>(x) / 100.0, static_cast<float>(y) / 100.0, static_cast<float>(z) / 100.0);
          break;
      }
      break;
  }
}

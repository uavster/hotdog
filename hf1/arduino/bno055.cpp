#include "core_pins.h"
#include "bno055.h"
#include "i2c_t3.h"
#include "timer.h"
#include "logger_interface.h"

#define DEVICE_ADDRESS    static_cast<uint8_t>(0x28)
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

static void ReadFromI2C(uint8_t register_address, void *data, uint8_t length) {
	Wire.beginTransmission(DEVICE_ADDRESS);	
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
	Wire.requestFrom(DEVICE_ADDRESS, length);
  auto bytes = reinterpret_cast<uint8_t *>(data);
  const auto start_ns = GetTimerNanoseconds();
  const TimerNanosType timeout_ns = length * kReadTimeoutPerByteNs + kReadTimeoutConstantNs;
	while(Wire.available() && GetTimerNanoseconds() - start_ns < timeout_ns) {
		*(bytes++) = Wire.read();
    --length;
	}
  ASSERTM(length == 0, "ReadFromI2C ERROR: device responded by the timeout expired reading data.");
}

static void WriteToI2C(uint8_t register_address, const void *data, uint8_t length) {
	Wire.beginTransmission(DEVICE_ADDRESS);
	Wire.write(register_address);	
  auto bytes = reinterpret_cast<const uint8_t *>(data);
	for(uint8_t i = 0; i < length; ++i) {
		Wire.write(*bytes);
		++bytes;
	}
	Wire.endTransmission();
	SleepForNanos(150000);
}

static uint8_t ReadByteFromI2C(uint8_t reg_addr) {
  uint8_t data;
  ReadFromI2C(reg_addr, &data, 1);
  return data;
}

static void WriteByteToI2C(uint8_t reg_addr, uint8_t value) {
  WriteToI2C(reg_addr, &value, 1);
}

static uint8_t SetBytePart(uint8_t dest, uint8_t offset, uint8_t mask, uint8_t value) {
  return (dest & (~mask)) | ((value & mask) << offset);
}

static uint8_t GetBytePart(uint8_t source, uint8_t offset, uint8_t mask) {
  return (source & mask) >> offset;
}

static void WritePageID(uint8_t page_id) {
  WriteByteToI2C(PAGE_ID_REG, page_id); 
}

static BNO055OperationMode GetOperationMode() {
  WritePageID(0);
  return static_cast<BNO055OperationMode>(GetBytePart(ReadByteFromI2C(OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK));
}

static void SetOperationMode(BNO055OperationMode op_mode) {
  if (GetOperationMode() == BNO055_OPERATION_MODE_CONFIG) {
    WriteByteToI2C(
      OPERATION_MODE_REG, 
      SetBytePart(ReadByteFromI2C(OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK, op_mode)
    );
  } else {
    WriteByteToI2C(
      OPERATION_MODE_REG, 
      SetBytePart(ReadByteFromI2C(OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK, BNO055_OPERATION_MODE_CONFIG) 
    );
    if (op_mode != BNO055_OPERATION_MODE_CONFIG) {
      WriteByteToI2C(
        OPERATION_MODE_REG, 
        SetBytePart(ReadByteFromI2C(OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK, op_mode)
      );
    }
  }
}

bool BNO055::begin(BNO055OperationMode op_mode) {
  // Hard reset.
  pinMode(27, OUTPUT);
  digitalWrite(27, 0);
  SleepForSeconds(0.01);
  pinMode(27, INPUT);
  SleepForNanos(650'000'000);  // Typical POR time is 650 ms.

  Wire.begin();
  SleepForNanos(1'000'000);
  
  // Wait for the chip to start up and check that we have the right model.
  SleepForNanos(400'000'000); // Typical start-up time is 400 ms.
  int timeout_ms = 200;  
  while(ReadByteFromI2C(CHIP_ID_REG) != EXPECTED_CHIP_ID && timeout_ms > 0) {
    SleepForNanos(10'000'000);
    timeout_ms -= 10;
  }
  ASSERTM(timeout_ms > 0, "IMU does not start or it's not a BNO055.");

  // Reset the chip in case begin() has been called a second time, 
  // or the MCU was reset by the programmer without power-cycling the IMU.
  SetOperationMode(BNO055_OPERATION_MODE_CONFIG);
  WriteByteToI2C(SYS_TRIGGER_REG, 0x20);

  // Check that we have the right chip. Retry until it is on.
  SleepForNanos(650'000'000);  // Typical POR time is 650 ms.
  timeout_ms = 200;
  while(ReadByteFromI2C(CHIP_ID_REG) != EXPECTED_CHIP_ID && timeout_ms > 0) {
    SleepForNanos(10'000'000);
    timeout_ms -= 10;
  }
  ASSERTM(timeout_ms > 0, "IMU does not start or it's not a BNO055.");
  SleepForNanos(50'000'000);

  // Configure external crystal.
  SetOperationMode(BNO055_OPERATION_MODE_CONFIG);
  SleepForNanos(25'000'000);
  WritePageID(0);
  WriteByteToI2C(SYS_TRIGGER_REG, 0x80);
  SleepForNanos(600'000'000);
  timeout_ms = 1000;
  while((ReadByteFromI2C(SYS_CLK_STATUS_REG) & 1) && timeout_ms > 0) {
    SleepForNanos(10'000'000);
    timeout_ms -= 10;
  }
  ASSERTM(!(ReadByteFromI2C(SYS_CLK_STATUS_REG) & 1), "Timed out waiting for ST_MAIN_CLK to become low.");
  ASSERTM(ReadByteFromI2C(SYS_TRIGGER_REG) & 0x80, "Unable to start external oscillator.");

  setMode(op_mode);
  SleepForNanos(20'000'000);

  return true;
}

void BNO055::setMode(BNO055OperationMode op_mode) {
  SetOperationMode(op_mode);
  last_mode_ = op_mode;
}

Vector<3> BNO055::getVector(TVectorType vector_type) const {
  // Assume that page 0 is selected.
  uint8_t buffer[6];
  ReadFromI2C(static_cast<uint8_t>(vector_type), buffer, sizeof(buffer) / sizeof(buffer[0]));
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

BNO055::CalibrationStatus BNO055::GetCalibrationStatus() const {
  const uint8_t reg = ReadByteFromI2C(CALIBRATION_STATUS_REG);  
  return {
    .operation_mode = GetOperationMode(),
    .system = static_cast<uint8_t>((reg >> 6) & 0b11),
    .gyroscopes = static_cast<uint8_t>((reg >> 4) & 0b11),
    .accelerometers = static_cast<uint8_t>((reg >> 2) & 0b11),
    .magnetometer = static_cast<uint8_t>(reg & 0b11)
  };
}

BNO055::CalibrationData BNO055::GetCalibrationData() const {
  SetOperationMode(BNO055_OPERATION_MODE_CONFIG);
  CalibrationData calibration_data;
  ReadFromI2C(SENSOR_OFFSETS_BASE_ADDRESS, &calibration_data, SENSOR_OFFSETS_NUM_REGISTERS);
  SetOperationMode(last_mode_);
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

void BNO055::SetCalibrationData(const CalibrationData &data) const {
  SetOperationMode(BNO055_OPERATION_MODE_CONFIG);
  SleepForNanos(25000000);
  WriteToI2C(SENSOR_OFFSETS_BASE_ADDRESS, &data, SENSOR_OFFSETS_NUM_REGISTERS);
  SetOperationMode(last_mode_);
}

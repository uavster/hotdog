#include "bno055.h"
#include "i2c_t3.h"
#include "timer.h"
#include "logger_interface.h"

#define DEVICE_ADDRESS    static_cast<uint8_t>(0x28)
#define EXPECTED_CHIP_ID  0xa0

#define CHIP_ID_REG             0

#define PAGE_ID_REG             0x7

#define OPERATION_MODE_POS			0
#define OPERATION_MODE_MASK			0Xf
#define OPERATION_MODE_REG			0x3d
#define SYS_TRIGGER_REG         0x3f
#define SYS_CLK_STATUS_REG      0x38

#define SET_SENSOR_OFFSETS_BASE_ADDRESS 0x55

static void ReadFromI2C(uint8_t register_address, uint8_t *data, uint8_t length) {
	Wire.beginTransmission(DEVICE_ADDRESS);	
	Wire.write(register_address);
	Wire.endTransmission();
  SleepForNanos(150000);
	Wire.requestFrom(DEVICE_ADDRESS, length);
	while(Wire.available()) {
		*(data++) = Wire.read();
	}
}

static void WriteToI2C(uint8_t register_address, const uint8_t *data, uint8_t length) {
	Wire.beginTransmission(DEVICE_ADDRESS);
	Wire.write(register_address);	
	for(uint8_t i = 0; i < length; ++i) {
		Wire.write(*data);
		++data;
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

static uint8_t GetOperationMode() {
  WritePageID(0);
  return GetBytePart(ReadByteFromI2C(OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK);
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
  while(ReadByteFromI2C(SYS_CLK_STATUS_REG) & 1) {
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

// TODO: Clean up interface to pass a packed structure.
void BNO055::setSensorOffsets(const uint8_t *calibration_data) {
  SetOperationMode(BNO055_OPERATION_MODE_CONFIG);
  SleepForNanos(25000000);

  WriteToI2C(SET_SENSOR_OFFSETS_BASE_ADDRESS, calibration_data, 22);

  SetOperationMode(last_mode_);
}

Vector<3> BNO055::getVector(TVectorType vector_type) {
  // Assume that page 0 is selected.
  uint8_t buffer[6];
  ReadFromI2C(static_cast<uint8_t>(vector_type), buffer, sizeof(buffer) / sizeof(buffer[0]));
  const int16_t x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  const int16_t y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  const int16_t z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  switch(vector_type) {
    case VECTOR_EULER:
      return Vector<3>(static_cast<float>(x) / 16.0, static_cast<float>(y) / 16.0, static_cast<float>(z) / 16.0);
    case VECTOR_LINEAR_ACCEL:
      return Vector<3>(static_cast<float>(x) / 100.0, static_cast<float>(y) / 100.0, static_cast<float>(z) / 100.0);
  }
  return Vector<3>();
}
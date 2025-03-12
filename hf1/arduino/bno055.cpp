#include "bno055.h"
#include <Wire.h>
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

#define PAGE_ZERO               0

#define SET_SENSOR_OFFSETS_BASE_ADDRESS 0x55

static void i2c_read(uint8_t register_address, uint8_t *data, uint8_t length) {
	Wire.beginTransmission(DEVICE_ADDRESS);	
	Wire.write(register_address);
	Wire.endTransmission();
  SleepForNanos(150000);
	Wire.requestFrom(DEVICE_ADDRESS, length);
	while(Wire.available()) {
		*(data++) = Wire.read();
	}
}

static void i2c_write(uint8_t register_address, const uint8_t *data, uint8_t length) {
	Wire.beginTransmission(DEVICE_ADDRESS);
	Wire.write(register_address);	
	for(uint8_t i = 0; i < length; ++i) {
		Wire.write(*data);
		++data;
	}
	Wire.endTransmission();
	SleepForNanos(150000);
}

static uint8_t i2c_read_byte(uint8_t reg_addr) {
  uint8_t data;
  i2c_read(reg_addr, &data, 1);
  return data;
}

static void i2c_write_byte(uint8_t reg_addr, uint8_t value) {
  i2c_write(reg_addr, &value, 1);
}

static uint8_t set_byte_part(uint8_t dest, uint8_t offset, uint8_t mask, uint8_t value) {
  return (dest & (~mask)) | ((value & mask) << offset);
}

static uint8_t get_byte_part(uint8_t source, uint8_t offset, uint8_t mask) {
  return (source & mask) >> offset;
}

static void write_page_id(uint8_t page_id) {
  i2c_write_byte(PAGE_ID_REG, page_id); 
}

static uint8_t get_operation_mode() {
  write_page_id(PAGE_ZERO);
  return get_byte_part(i2c_read_byte(OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK);
}

static void set_operation_mode(BNO055OperationMode op_mode) {
  if (get_operation_mode() == BNO055_OPERATION_MODE_CONFIG) {
    i2c_write_byte(
      OPERATION_MODE_REG, 
      set_byte_part(i2c_read_byte(OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK, op_mode)
    );
  } else {
    i2c_write_byte(
      OPERATION_MODE_REG, 
      set_byte_part(i2c_read_byte(OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK, BNO055_OPERATION_MODE_CONFIG) 
    );
    if (op_mode != BNO055_OPERATION_MODE_CONFIG) {
      i2c_write_byte(
        OPERATION_MODE_REG, 
        set_byte_part(i2c_read_byte(OPERATION_MODE_REG), OPERATION_MODE_POS, OPERATION_MODE_MASK, op_mode)
      );
    }
  }
}

bool BNO055::begin(BNO055OperationMode op_mode) {
  Wire.begin();
  
  SleepForNanos(1000000);
  
  // TODO: We normally don't need it now because of the initial delay after boot, but ensure we retry if that delay changes. 

  // Check that we have the right chip.
  ASSERTM(i2c_read_byte(CHIP_ID_REG) == EXPECTED_CHIP_ID, "Unknown chip ID.");

  // TODO: Reset the chip in case begin() is called a second time.

  // TODO: configure power mode.

  // TODO: Configure external crystal.
  // set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
  // SleepForNanos(25000000);
  // write_page_id(PAGE_ZERO);
  // i2c_write_byte(SYS_TRIGGER_REG, 0x80);
  // SleepForNanos(10000000);

  setMode(op_mode);
  SleepForNanos(20000000);

  LOG_INFO("BNO055 init ok.\n");
  return true;
}

void BNO055::setMode(BNO055OperationMode op_mode) {
  set_operation_mode(op_mode);
  last_mode_ = op_mode;
}

// TODO: Clean up interface to pass a packed structure.
void BNO055::setSensorOffsets(const uint8_t *calibration_data) {
  set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
  SleepForNanos(25000000);

  i2c_write(SET_SENSOR_OFFSETS_BASE_ADDRESS, calibration_data, 22);

  set_operation_mode(last_mode_);
}

imu::Vector<3> BNO055::getVector(TVectorType vector_type) {
  // TODO: We assume that page 0 is selected, but we should make sure by setting it with caching, so we don't waste time setting it over and over again.
  uint8_t buffer[6];
  i2c_read(static_cast<uint8_t>(vector_type), buffer, sizeof(buffer) / sizeof(buffer[0]));
  const int16_t x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  const int16_t y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  const int16_t z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  switch(vector_type) {
    case VECTOR_EULER:
      return imu::Vector<3>(static_cast<double>(x) / 16.0, static_cast<double>(y) / 16.0, static_cast<double>(z) / 16.0);
    case VECTOR_LINEAR_ACCEL:
      return imu::Vector<3>(static_cast<double>(x) / 100.0, static_cast<double>(y) / 100.0, static_cast<double>(z) / 100.0);
  }
  return imu::Vector<3>();
}
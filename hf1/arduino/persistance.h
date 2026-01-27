#ifndef PERSISTANCE_INCLUDED__
#define PERSISTANCE_INCLUDED__

#include "i2c_t3.h"
#include "I2C_eeprom.h"
#include "status_or.h"

class PersistentStorage {
private:
  static constexpr uint8_t kEEPROMDeviceID = 0x50;

public:
  PersistentStorage() : eeprom_(kEEPROMDeviceID, &Wire) {}  
  void Init();
  template<typename ObjectT> StatusOr<ObjectT> get(int address);
  template<typename ObjectT> Status put(int address, const ObjectT &object);

private:
  I2C_eeprom eeprom_;
};

// Global persistent store. Must be instantiated in the sketch file.
extern PersistentStorage persistent_storage;

#include "persistance.hh"

#endif
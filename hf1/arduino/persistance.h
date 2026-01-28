#ifndef PERSISTANCE_INCLUDED__
#define PERSISTANCE_INCLUDED__

#include "i2c_t3.h"
#include "I2C_eeprom.h"
#include "status_or.h"

// Handles information storage in a persistent medium (currently, an external EEPROM).
// This class assumes the I2C bus is initialized by the caller.
class PersistentStorage {
private:
  static constexpr uint8_t kEEPROMDeviceID = 0x50;

public:
  PersistentStorage(i2c_t3 *wire) : eeprom_(kEEPROMDeviceID, wire) {}

  // Initializes the persistent storage.
  // Must be called from setup().
  // Precondition: the passed Wire object must be initialized before calling this function.
  void Init();

  // Reads an object of type `ObjectT` from `address`.
  template<typename ObjectT> StatusOr<ObjectT> get(int address);

  // Writes an object of type `ObjectT` from `address`.
  template<typename ObjectT> Status put(int address, const ObjectT &object);

  int capacity() const { return eeprom_.getDeviceSize(); }
  int first_address() const { return 0; }
  int last_address() const { return capacity() - 1; }

private:
  I2C_eeprom eeprom_;
};

// Global persistent store. Must be instantiated in the sketch file.
extern PersistentStorage persistent_storage;

#include "persistance.hh"

#endif
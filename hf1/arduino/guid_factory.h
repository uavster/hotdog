#ifndef GUID_ARDUINO_
#define GUID_ARDUINO_

#include "guid_factory_interface.h"

// Arduino implementation of the Global Unique IDentifier factory.
class GUIDFactory : public GUIDFactoryInterface {
public:
  GUIDFactory();
  virtual void CreateGUID(int len, uint8_t *buffer, uint8_t max_byte_value);
};

#endif  // GUID_ARDUINO_
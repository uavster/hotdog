#ifndef GUID_LINUX_
#define GUID_LINUX_

#include "guid_factory_interface.h"

// Linux implementation of the Global Unique IDentifier factory.
class GUIDFactory : public GUIDFactoryInterface {
public:
  GUIDFactory();
  virtual void CreateGUID(int len, uint8_t *buffer, uint8_t max_byte_value);
};

#endif  // GUID_LINUX_

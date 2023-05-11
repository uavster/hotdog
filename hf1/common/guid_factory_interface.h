#ifndef GUID_INTERFACE_
#define GUID_INTERFACE_

#include <stdint.h>
#include "packed_number.h"

// Global Unique IDentifier.
template<int kNumBytes, int kMaxValuePerByte> class GUID {
  friend class GUIDFactoryInterface;
public:
  using GUIDType = PackedInteger<kNumBytes, kMaxValuePerByte>;
  operator GUIDType() const { return guid_; }

protected:
  GUID() {}
  GUIDType &guid() { return guid_; }
  const GUIDType &guid() const { return guid_; }

private:
  GUIDType guid_;
};

// GUID factory interface.
// Virtuals must be implemented on each platform.
class GUIDFactoryInterface {
public:
  virtual void CreateGUID(int len, uint8_t *buffer, uint8_t max_byte_value) = 0;

  template<int kNumBytes, int kMaxValuePerByte> GUID<kNumBytes, kMaxValuePerByte> CreateGUID() {
    GUID<kNumBytes, kMaxValuePerByte> guid;
    CreateGUID(kNumBytes, guid.guid().bytes, kMaxValuePerByte);
    return guid;
  }
};

#endif  // GUID_INTERFACE_
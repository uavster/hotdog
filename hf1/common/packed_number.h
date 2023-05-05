#ifndef PACKED_NUMBER_
#define PACKED_NUMBER_

#include <stdint.h>

template<int kNumBytes, int kMaxValuePerByte> struct PackedInteger {
#pragma pack(push, 1)
  uint8_t bytes[kNumBytes];
#pragma pack(pop)

  PackedInteger(uint64_t n) { *this = n; }
  PackedInteger() : PackedInteger(0) {}

  operator uint64_t() const {
    uint64_t n = 0;
    uint64_t multiplier = 1;
    for (int i = 0; i < kNumBytes; ++i) {
      n += bytes[i] * multiplier;
      multiplier *= kMaxValuePerByte;
    }
    return n;
  }

  PackedInteger &operator=(uint64_t n) {    
    for (int i = 0; i < kNumBytes; ++i) {
      bytes[i] = n % kMaxValuePerByte;
      n /= kMaxValuePerByte;
    }
    return *this;
  }

  const uint8_t *operator&() const {
    return bytes;
  }
};

#endif  // PACKED_NUMBER_
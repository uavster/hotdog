#ifndef NETWORK__
#define NETWORK__

#include <stdint.h>

enum Endianness { kLittleEndian, kBigEndian };

uint16_t FlipBytes(uint16_t v);
uint16_t FlipBytes(uint32_t v);
int16_t FlipBytes(int16_t v);
int32_t FlipBytes(int32_t v);

namespace {
  template<Endianness PlatformEndianness, typename DataType> inline DataType n2l(DataType v) { return v; }
  template<> inline uint16_t n2l<Endianness::kBigEndian, uint16_t>(uint16_t v) { return FlipBytes(v); }
  template<> inline uint32_t n2l<Endianness::kBigEndian, uint32_t>(uint32_t v) { return FlipBytes(v); }
  template<> inline int16_t n2l<Endianness::kBigEndian, int16_t>(int16_t v) { return FlipBytes(v); }
  template<> inline int32_t n2l<Endianness::kBigEndian, int32_t>(int32_t v) { return FlipBytes(v); }

  template<Endianness PlatformEndianness, typename DataType> inline DataType l2n(DataType v) { return v; }
  template<> inline uint16_t l2n<Endianness::kBigEndian, uint16_t>(uint16_t v) { return FlipBytes(v); }
  template<> inline uint32_t l2n<Endianness::kBigEndian, uint32_t>(uint32_t v) { return FlipBytes(v); }
  template<> inline int16_t l2n<Endianness::kBigEndian, int16_t>(int16_t v) { return FlipBytes(v); }
  template<> inline int32_t l2n<Endianness::kBigEndian, int32_t>(int32_t v) { return FlipBytes(v); }
}

template<Endianness PlatformEndianness, typename DataType> DataType NetworkToLocal(DataType v) { return n2l<PlatformEndianness, DataType>(v); }
template<Endianness PlatformEndianness, typename DataType> DataType LocalToNetwork(DataType v) { return l2n<PlatformEndianness, DataType>(v); }

#endif  // NETWORK__
#ifndef NETWORK__
#define NETWORK__

#include <stdint.h>

enum Endianness { kLittleEndian, kBigEndian };

uint16_t FlipBytes(uint16_t v);
uint32_t FlipBytes(uint32_t v);
uint64_t FlipBytes(uint64_t v);
int16_t FlipBytes(int16_t v);
int32_t FlipBytes(int32_t v);
int64_t FlipBytes(int64_t v);

namespace {
  template<Endianness LocalEndianness, typename DataType> inline DataType n2l(DataType v) { return v; }
  template<> inline uint16_t n2l<Endianness::kBigEndian, uint16_t>(uint16_t v) { return FlipBytes(v); }
  template<> inline uint32_t n2l<Endianness::kBigEndian, uint32_t>(uint32_t v) { return FlipBytes(v); }
  template<> inline uint64_t n2l<Endianness::kBigEndian, uint64_t>(uint64_t v) { return FlipBytes(v); }
  template<> inline int16_t n2l<Endianness::kBigEndian, int16_t>(int16_t v) { return FlipBytes(v); }
  template<> inline int32_t n2l<Endianness::kBigEndian, int32_t>(int32_t v) { return FlipBytes(v); }
  template<> inline int64_t n2l<Endianness::kBigEndian, int64_t>(int64_t v) { return FlipBytes(v); }

  template<Endianness LocalEndianness, typename DataType> inline DataType l2n(DataType v) { return v; }
  template<> inline uint16_t l2n<Endianness::kBigEndian, uint16_t>(uint16_t v) { return FlipBytes(v); }
  template<> inline uint32_t l2n<Endianness::kBigEndian, uint32_t>(uint32_t v) { return FlipBytes(v); }
  template<> inline uint64_t l2n<Endianness::kBigEndian, uint64_t>(uint64_t v) { return FlipBytes(v); }
  template<> inline int16_t l2n<Endianness::kBigEndian, int16_t>(int16_t v) { return FlipBytes(v); }
  template<> inline int32_t l2n<Endianness::kBigEndian, int32_t>(int32_t v) { return FlipBytes(v); }
  template<> inline int64_t l2n<Endianness::kBigEndian, int64_t>(int64_t v) { return FlipBytes(v); }
}

template<Endianness LocalEndianness, typename DataType> DataType NetworkToLocal(DataType v) { return n2l<LocalEndianness, DataType>(v); }
template<Endianness LocalEndianness, typename DataType> DataType LocalToNetwork(DataType v) { return l2n<LocalEndianness, DataType>(v); }

#endif  // NETWORK__
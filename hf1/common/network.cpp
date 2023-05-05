#include "network.h"

uint16_t FlipBytes(uint16_t v) {
  uint8_t *p = reinterpret_cast<uint8_t *>(&v);
  uint8_t tmp = p[0];
  p[0] = p[1];
  p[1] = tmp;
  return *reinterpret_cast<uint16_t *>(p);
}

uint32_t FlipBytes(uint32_t v) {
  uint8_t *p = reinterpret_cast<uint8_t *>(&v);
  uint8_t tmp = p[0];
  p[0] = p[3];
  p[3] = tmp;
  tmp = p[1];
  p[1] = p[2];
  p[2] = tmp;
  return *reinterpret_cast<uint32_t *>(p);
}

uint64_t FlipBytes(uint64_t v) {
  uint8_t *p = reinterpret_cast<uint8_t *>(&v);
  uint8_t tmp = p[0];
  p[0] = p[7];
  p[7] = tmp;
  tmp = p[1];
  p[1] = p[6];
  p[6] = tmp;
  tmp = p[2];
  p[2] = p[5];
  p[5] = tmp;
  tmp = p[3];
  p[3] = p[4];
  p[4] = tmp;
  return *reinterpret_cast<uint64_t *>(p);
}

int16_t FlipBytes(int16_t v) {
  return FlipBytes(static_cast<uint16_t>(v));
}

int32_t FlipBytes(int32_t v) {
  return FlipBytes(static_cast<uint32_t>(v));
}

int64_t FlipBytes(int64_t v) {
  return FlipBytes(static_cast<uint64_t>(v));
}
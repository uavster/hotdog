#include "utils.h"
#include <math.h>
#include <algorithm>

void Uint64ToString(uint64_t number, char *str) {
  int str_length = 1;
  if (number >= 10) {
    str_length = log10(number) + 1;
  }
  for (int i = str_length - 1; i >= 0; --i) {
    str[i] = '0' + (number % 10);
    number /= 10;
  }
  str[str_length] = '\0';
}

float NormalizeRadians(float radians) {
  return remainderf(radians, 2 * M_PI);
}

int IndexMod(int index, int array_size) {
  if (array_size >= 0) {
    return index >= 0 ? index % array_size : array_size - (-index % array_size);
  } else {
    return index <= 0 ? index % array_size : array_size + (index % array_size);
  }
}

float IndexModf(float index, float array_size) {
  if (array_size >= 0) {
    return index >= 0 ? fmodf(index, array_size) : array_size - fmodf(-index, array_size);
  } else {
    return index <= 0 ? fmodf(index, array_size) : array_size + fmodf(index, array_size);
  }  
}
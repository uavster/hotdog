#include <Arduino.h>
#include "guid_factory.h"

GUIDFactory::GUIDFactory() {
  static bool is_init = false;
  if (!is_init) {
    randomSeed(analogRead(0));
    is_init = true;
  }
}

void GUIDFactory::CreateGUID(int len, uint8_t *buffer, uint8_t max_byte_value) {
  for (int i = 0; i < len; ++i) {
    buffer[i] = random(max_byte_value);
  }
}

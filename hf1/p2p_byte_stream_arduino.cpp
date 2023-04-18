#include "p2p_byte_stream_arduino.h"
#include <algorithm>

Stream &P2PByteStreamArduino::stream() const {
  return *static_cast<Stream *>(handler().object);
}

int P2PByteStreamArduino::Write(const void *buffer, int length) {
  return stream().write(static_cast<const uint8_t *>(buffer), std::min(length, stream().availableForWrite()));
}

int P2PByteStreamArduino::Read(void *buffer, int length) {
  return stream().readBytes(static_cast<uint8_t *>(buffer), std::min(length, stream().available()));
}

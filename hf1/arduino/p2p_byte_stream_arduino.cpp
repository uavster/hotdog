#include "p2p_byte_stream_arduino.h"
#include <algorithm>
#include "Arduino.h"

Stream &P2PByteStreamArduino::stream() const {
  return *static_cast<Stream *>(handler().object);
}

int P2PByteStreamArduino::Write(const void *buffer, int length) {
  return stream().write(static_cast<const uint8_t *>(buffer), std::min(length, stream().availableForWrite()));
}

int P2PByteStreamArduino::Read(void *buffer, int length) {
  noInterrupts();
  if (UART0_S1 & UART_S1_FE) {
    // There was a framing error due to a reset or link interruptio. Clear the flag so
    // the uart continues receiving, and let our protocol skip the errors.
    uint8_t uart_d = UART0_D;
    ++uart_d;
  }
  interrupts();
  return stream().readBytes(static_cast<uint8_t *>(buffer), std::min(length, stream().available()));
}

int P2PByteStreamArduino::GetBurstMaxLength() {
  return 56;
}

int P2PByteStreamArduino::GetBurstIngestionNanosecondsPerByte() {
  return 64000;
}

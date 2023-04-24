#include "p2p_byte_stream_linux.h"
#include <unistd.h>

int P2PByteStreamLinux::Write(const void *buffer, int length) {
  return write(handler().fd, buffer, length);
}

int P2PByteStreamLinux::Read(void *buffer, int length) {
  return read(handler().fd, buffer, length);
}


#include "p2p_byte_stream_interface.h"

class P2PByteStreamLinux : public P2PByteStreamInterface<kLittleEndian> {
public:
  // Does not take ownership of the file descriptor, which must outlive this object.
  P2PByteStreamLinux(int fd, std::function<void()> &&physical_io_error_callback) 
    : P2PByteStreamInterface<kLittleEndian>(Handler{ .fd = fd}, std::move(physical_io_error_callback)) {}

  virtual int Write(const void *buffer, int length);
  virtual int Read(void *buffer, int length);
  virtual int GetBurstMaxLength();
  virtual int GetBurstIngestionNanosecondsPerByte();
  virtual int GetAtomicSendMaxLength();
};

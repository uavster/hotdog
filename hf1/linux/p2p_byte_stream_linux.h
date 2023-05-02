#include "p2p_byte_stream_interface.h"

class P2PByteStreamLinux : public P2PByteStreamInterface<kLittleEndian> {
public:
  // Does not take ownership of the stream, which must outlive this object.
  P2PByteStreamLinux(int fd) 
    : P2PByteStreamInterface<kLittleEndian>(Handler{ .fd = fd}) {}

  virtual int Write(const void *buffer, int length);
  virtual int Read(void *buffer, int length);
  virtual int GetBurstMaxLength();
  virtual int GetBurstIngestionNanosecondsPerByte();
  virtual int GetAtomicSendMaxLength();
};

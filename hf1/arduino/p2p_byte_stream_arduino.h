#include "p2p_byte_stream_interface.h"
#include "Stream.h"

class P2PByteStreamArduino : public P2PByteStreamInterface<kLittleEndian> {
public:
  // Does not take ownership of the stream, which must outlive this object.
  P2PByteStreamArduino(Stream *stream) 
    : P2PByteStreamInterface<kLittleEndian>(Handler{ .object = stream}) {}

  virtual int Write(const void *buffer, int length);
  virtual int Read(void *buffer, int length);
  virtual int GetBurstIngestionNanosecondsPerByte();

protected:
  Stream &stream() const;
};
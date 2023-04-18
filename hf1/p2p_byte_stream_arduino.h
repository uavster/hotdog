#include "p2p_byte_stream_interface.h"
#include "Stream.h"

class P2PByteStreamArduino : public P2PByteStreamInterface<kLittleEndian> {
public:
  virtual int Write(const void *buffer, int length);
  virtual int Read(void *buffer, int length);

protected:
  Stream &stream() const;
};
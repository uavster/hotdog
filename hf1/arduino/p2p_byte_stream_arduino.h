// #ifndef P2P_BYTE_STREAM_ARDUINO_
// #define P2P_BYTE_STREAM_ARDUINO_

#include "p2p_byte_stream_interface.h"
#include "Stream.h"

class P2PByteStreamArduino : public P2PByteStreamInterface<kLittleEndian> {
public:
  // Does not take ownership of the stream, which must outlive this object.
  P2PByteStreamArduino(Stream *stream) 
    : P2PByteStreamInterface<kLittleEndian>(Handler{ .object = stream}) {}

  virtual int Write(const void *buffer, int length);
  virtual int Read(void *buffer, int length);
  virtual int GetBurstMaxLength();
  virtual int GetBurstIngestionNanosecondsPerByte();
  virtual int GetAtomicSendMaxLength();

protected:
  Stream &stream() const;
};

// #endif  // P2P_BYTE_STREAM_ARDUINO_
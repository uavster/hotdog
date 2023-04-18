#ifndef P2P_BYTE_STREAM_INTERFACE__
#define P2P_BYTE_STREAM_INTERFACE__

#include <stdint.h>
#include "network.h"

// Interface to send and receive byte strings over a point-to-point link.
// Must be implemented on each platform.

#define kMaxPortNameLength 64

template<Endianness LocalEndianness> class P2PByteStreamInterface {
public:
  typedef union {
    // In plaforms like Arduino, this points to an object handling the stream.
    void *object;
    // In platform like Unix/Linux, this is the file descriptor of the stream.
    int fd;
  } Handler;

  // Creates a byte transfer object with the platform-specific handler.
  // The handler must have been initialized with the proper configuration before this call.
  P2PByteStreamInterface(Handler handler) : handler_(handler) {}

  // Attempts to send a maximum of `length` bytes in the buffer through the link, and returns the 
  // number of bytes actually sent. This call never blocks.
  virtual int Write(const void *buffer, int length) = 0;

  // Attempts to receive a maximum of `length` bytes in the buffer through the link, and returns the 
  // number of bytes actually received. This call never blocks.
  virtual int Read(void *buffer, int length) = 0;
    
  uint8_t ReadByteOrDefault(uint8_t default_output = 0xff) { 
    uint8_t c;
    return Read(&c, 1) == 1 ? c : default_output;
  }

  template<typename DataType> DataType NetworkToLocal(uint8_t v) { return NetworkToLocal<LocalEndianness, DataType>(v); }
  template<typename DataType> DataType LocalToNetwork(uint8_t v) { return LocalToNetwork<LocalEndianness, DataType>(v); }

protected:
  const Handler &handler() const { return handler_; }

private:
  Handler handler_;
};

#endif  // P2P_BYTE_STREAM_INTERFACE__
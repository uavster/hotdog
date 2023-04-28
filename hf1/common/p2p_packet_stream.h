// A buffered stream of P2P packets.
// 
// The Each pair of 

#include "p2p_packet_protocol.h"
#include "p2p_byte_stream_interface.h"
#include "ring_buffer.h"
#include "status_or.h"
#ifdef ARDUINO
#include <DebugLog.h>
#define assert ASSERT
#else
#include <assert.h>
#endif

class P2PPacket {
public:
  P2PPacket() {
    data_.header.start_token = kP2PStartToken;
  }

  P2PHeader *header() { return &data_.header; }
  const P2PHeader *header() const { return &data_.header; }

  uint8_t length() const { return data_.header.length; }
  uint8_t &length() { return data_.header.length; }

  const uint8_t *content() const { return data_.content_and_footer; }
  uint8_t *content() { return data_.content_and_footer; }

  // Decodes the content in place and updates the length accordingly. 
  // Returns true if success, or false if error.
  // An error will occur if the encoded content is malformed.
  bool PrepareToRead();

  // Encodes the content in place and updates the length and checksum accordingly.
  // Returns true if success, or false if error.
  // An error will occur if the encoded content surpasses the maximum content length. 
  bool PrepareToSend();

  P2PChecksumType checksum() const { return *reinterpret_cast<const P2PChecksumType *>(&data_.content_and_footer[length() + offsetof(P2PFooter, checksum)]); }
  P2PChecksumType &checksum() { return *reinterpret_cast<P2PChecksumType *>(&data_.content_and_footer[length() + offsetof(P2PFooter, checksum)]); }

protected:
  P2PChecksumType CalculateChecksum() const; 

private:
#pragma pack(push, 1)
  struct {
    P2PHeader header;
    uint8_t content_and_footer[kP2PMaxContentLength + sizeof(P2PFooter)];
  } data_;
  //__attribute__ ((aligned (4)));  // Aligning the memory ensures that we can access it as a byte array.
#pragma pack(pop)
};

class P2PPacketView {
public:
  // Does not take ownership of the packet, which must outlive this object.
  P2PPacketView(const P2PPacket *packet) : packet_(packet) {}
  P2PPacketView() : P2PPacketView(NULL) {}

  uint8_t length() const { 
    assert(packet_ != NULL);
    return packet_->length();
  }
  const uint8_t *content() {
    assert(packet_ != NULL);
    return packet_->content();
  }

private:
  const P2PPacket *packet_;
};

class P2PMutablePacketView {
public:
  // Does not take ownership of the packet, which must outlive this object.
  P2PMutablePacketView(P2PPacket *packet) : packet_(packet) {}
  P2PMutablePacketView() : P2PMutablePacketView(NULL) {}

  uint8_t length() const { 
    assert(packet_ != NULL);
    return packet_->length();
  }
  uint8_t &length() { 
    assert(packet_ != NULL);
    return packet_->length();
  }

  const uint8_t *content() const { 
    assert(packet_ != NULL);
    return packet_->content();
  }
  uint8_t *content() { 
    assert(packet_ != NULL);
    return packet_->content();
  }

private:
  P2PPacket *packet_;
};

template<int kCapacity, Endianness LocalEndianness> class P2PPacketInputStream {
public:
  // Does not take ownership of the byte stream, which must outlive this object.
  P2PPacketInputStream(P2PByteStreamInterface<LocalEndianness> *byte_stream) : byte_stream_(*byte_stream), state_(kWaitingForPacket) {}

  // Returns the number of times that Consume() can be called without OldestPacket() returning NULL.
  int NumAvailablePackets() { return packet_buffer_.Size(); }

  // Returns a view to the oldest packet in the stream, or kUnavailableError if empty.
  StatusOr<P2PPacketView> OldestPacket() { 
    const P2PPacket *packet = packet_buffer_.OldestValue();
    if (packet == NULL) {
      return Status::kUnavailableError;
    }
    return P2PPacketView(packet);
  }

  // Consumes the oldest packet in the stream. Afterwards, OldestPacket() returns a new value.
  // Returns false, if there is no packet to consume.
  bool Consume() { return packet_buffer_.Consume(); }

  void Run();

private:
  RingBuffer<P2PPacket, kCapacity> packet_buffer_;
  P2PByteStreamInterface<LocalEndianness> &byte_stream_;
  unsigned int current_field_read_bytes_;
  enum State { kWaitingForPacket, kReadingHeader, kReadingContent, kDisambiguatingStartTokenInContent, kReadingFooter } state_;
};

template<int kCapacity, Endianness LocalEndianness> class P2PPacketOutputStream {
public:
  // Does not take ownership of the byte stream, which must outlive this object.
  P2PPacketOutputStream(P2PByteStreamInterface<LocalEndianness> *byte_stream) : byte_stream_(*byte_stream), state_(kGettingNextPacket) {}

  // Returns the number of packet slots available for writing in the stream.
  int NumAvailableSlots() { return packet_buffer_.Capacity() - packet_buffer_.Size(); }

  // Returns a view to a new packet in the stream, or kUnavailableError if no space is available in the stream.
  // Commit() must be called for the packet to be finalized.
  StatusOr<P2PMutablePacketView> NewPacket() { 
    if (NumAvailableSlots() == 0) {
      return Status::kUnavailableError;
    }
    return P2PMutablePacketView(&packet_buffer_.NewValue());
  }

  // Commits changes to the new packet. Must be called for the packet to be sent.
  // Afterwards, NewPacket() returns a new value.
  bool Commit() {
    if (!packet_buffer_.NewValue().PrepareToSend()) { return false; }
    // Fix endianness.
    packet_buffer_.NewValue().checksum() = LocalToNetwork<LocalEndianness>(packet_buffer_.NewValue().checksum());
    packet_buffer_.NewValue().length() = LocalToNetwork<LocalEndianness>(packet_buffer_.NewValue().length());

    packet_buffer_.Commit();
    return true;
  }

  void Run(uint64_t timestamp_ns);

private:
  RingBuffer<P2PPacket, kCapacity> packet_buffer_;
  P2PByteStreamInterface<LocalEndianness> &byte_stream_;
  int total_packet_length_;
  int pending_packet_bytes_;
  uint64_t burst_end_timestamp_ns_;
  enum State { kGettingNextPacket, kSendingPacket, kWaitingForOtherEnd } state_;  
};

// template<int kCapacity, Endianness LocalEndianness> class P2PPacketStream {

#include "p2p_packet_stream.hh"

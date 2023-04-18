// A buffered stream of P2P packets.
// 
// The Each pair of 

#include "p2p_packet_stream_protocol.h"
#include "p2p_byte_stream_interface.h"
#include "ring_buffer.h"

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

  // Encodes the content in place and updates the length accordingly. Returns true if success, or false if error.
  // An error will occur if the encoded content is malformed.
  bool PrepareToRead();

  // Decodes the content in place and updates the length and checksum accordingly. Returns true if success, or false if error.
  // An error will occur if the encoded content surpasses the maximum content length. 
  bool PrepareToSend();

  P2PChecksumType checksum() const { return *reinterpret_cast<const P2PChecksumType *>(&data_.content_and_footer[length() + offsetof(P2PFooter, checksum)]); }
  P2PChecksumType &checksum() { return *reinterpret_cast<P2PChecksumType *>(&data_.content_and_footer[length() + offsetof(P2PFooter, checksum)]); }

  int TotalLength() const { return sizeof(P2PHeader) + length() + sizeof(P2PFooter); }

protected:
  P2PChecksumType CalculateChecksum() const; 

private:
#pragma pack(push, 1)
  struct {
    P2PHeader header;
    uint8_t content_and_footer[kP2PMaxContentLength + sizeof(P2PFooter)];
  } data_;
#pragma pack(pop)
};

class P2PPacketView {
public:
  // Does not take ownership of the packet, which must outlive this object.
  P2PPacketView(const P2PPacket *packet) : packet_(*packet) {}

  uint8_t length() const { return packet_.length(); }
  const uint8_t *content() { return packet_.content(); }

private:
  const P2PPacket &packet_;
};

class P2PMutablePacketView {
public:
  // Does not take ownership of the packet, which must outlive this object.
  P2PMutablePacketView(P2PPacket *packet) : packet_(*packet) {}

  uint8_t length() const { return packet_.length(); }
  uint8_t &length() { return packet_.length(); }

  const uint8_t *content() const { return packet_.content(); }
  uint8_t *content() { return packet_.content(); }

  // Commits changes to the content and its length. Must be called before sending the packet.
  bool Commit() { return packet_.PrepareToSend(); }
  
private:
  P2PPacket &packet_;
};

enum Status {
  kSuccess, kUnavailableError
};

template<typename ValueType> class StatusOr {
public:
  StatusOr(ValueType &&v) : status_(kSuccess), value_(v) {}
  StatusOr(Status e) : status_(e) {}
  
  ValueType *operator->() { return value_; }
  ValueType &operator*() { return value_; }

  bool ok() const { return status_ == kSuccess; }

private:
  Status status_;
  ValueType value_;
};

template<int kCapacity, Endianness LocalEndianness> class P2PPacketInputStream {
public:
  P2PPacketInputStream(P2PByteStreamInterface<LocalEndianness> &byte_stream) : byte_stream_(byte_stream), state_(kWaitingForPacket) {}

  // Returns the number of times that Consume() can be called without OldestPacket() returning NULL.
  int NumAvailablePackets() { return packet_buffer_.size(); }

  // Returns a view to the oldest packet in the stream, or kUnavailableError if empty.
  StatusOr<P2PPacketView> OldestPacket() { 
    const P2PPacket *packet = packet_buffer_.OldestValue();
    if (packet == NULL) {
      return Status::kUnavailableError;
    }
    return P2PPacketView(*packet);
  }

  // Consumes the oldest packet in the stream. Afterwards, OldestPacket() returns a new value.
  void Consume() { return packet_buffer_.Consume(); }

  void Run();

private:
  RingBuffer<P2PPacket, kCapacity> packet_buffer_;
  P2PByteStreamInterface<LocalEndianness> &byte_stream_;
  int current_field_read_bytes_;
  enum State { kWaitingForPacket, kVerifyingStartToken, kReadingHeader, kReadingContent, kReadingFooter } state_;
};

template<int kCapacity, Endianness LocalEndianness> class P2PPacketOutputStream {
public:
  P2PPacketOutputStream(P2PByteStreamInterface<LocalEndianness> &byte_stream) : byte_stream_(byte_stream), state_(kGettingNextPacket) {}

  // Returns a view to a new packet in the stream, or kUnavailableError if no space is available in the stream.
  // Commit() must be called for the packet to be finalized.
  StatusOr<P2PMutablePacketView> NewPacket() { 
    if (packet_buffer_.Size() >= packet_buffer_.Capacity()) {
      return Status::kUnavailableError;
    }
    return P2PMutablePacketView(&packet_buffer_.NewValue());
  }

  bool Commit() {
    StatusOr<P2PMutablePacketView> maybe_new_packet_view = packet_buffer_.NewPacket();
    if (!maybe_new_packet_view.ok()) {
      return false;
    }
    maybe_new_packet_view->Commit();
    return packet_buffer_.Commit();
  }

  void Run();

private:
  RingBuffer<P2PPacket, kCapacity> packet_buffer_;
  P2PByteStreamInterface<LocalEndianness> &byte_stream_;
  int pending_packet_bytes;
  enum State { kGettingNextPacket, kSendingPacket } state_;  
};

// template<int kCapacity, Endianness LocalEndianness> class P2PPacketStream {

#include "p2p_packet_stream.hh"

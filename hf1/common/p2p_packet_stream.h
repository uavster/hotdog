// A buffered stream of P2P packets.
// 
// The Each pair of 

#include "p2p_packet_protocol.h"
#include "p2p_byte_stream_interface.h"
#include "priority_ring_buffer.h"
#include "status_or.h"
#include "timer_interface.h"
#ifdef ARDUINO
#include <DebugLog.h>
#define assert ASSERT
#else
#include <assert.h>
#endif

class P2PPriority {
public:
  enum Level { kReserved = 0, kHigh, kMedium, kLow, kNumLevels };

  P2PPriority(Level level) : level_(level) {}  

  P2PPriority(int numeric_priority) : level_(static_cast<Level>(numeric_priority)) {
    assert(level_ < kNumLevels);
  }
  virtual operator int() const { return static_cast<int>(level_); }

  bool operator==(const P2PPriority &other) { return static_cast<int>(*this) == static_cast<int>(other); }
  bool operator!=(const P2PPriority &other) { return !(*this == other); }
  bool operator<(const P2PPriority &other) { return static_cast<int>(*this) > static_cast<int>(other); }
  bool operator<=(const P2PPriority &other) { return *this < other || *this == other; }
  bool operator>(const P2PPriority &other) { return !(*this <= other); }
  bool operator>=(const P2PPriority &other) { return !(*this < other); }

private:
  Level level_;
};

class P2PPacket {
public:
  P2PPacket() : commit_time_ns_(-1ULL) {
    data_.header.start_token = kP2PStartToken;
  }

  P2PHeader *header() { return &data_.header; }
  const P2PHeader *header() const { return &data_.header; }

  uint8_t length() const { return data_.header.length; }
  uint8_t &length() { return data_.header.length; }

  const uint8_t *content() const { return data_.content_and_footer; }
  uint8_t *content() { return data_.content_and_footer; }

  const P2PSequenceNumberType &sequence_number() const { 
    return data_.header.sequence_number;
  }
  P2PSequenceNumberType &sequence_number() { 
    return data_.header.sequence_number;
  }

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

  uint64_t &commit_time_ns() { return commit_time_ns_; }
  uint64_t commit_time_ns() const { return commit_time_ns_; }

protected:
  P2PChecksumType CalculateChecksum() const; 

private:
#pragma pack(push, 1)
  struct {
    P2PHeader header;
    uint8_t content_and_footer[kP2PMaxContentLength + sizeof(P2PFooter)];
  } data_;
  // __attribute__ ((aligned (4)));  // Aligning the memory ensures that we can access it as a byte array.
#pragma pack(pop)

  // Attention: keep all metadata under data_ to not mess with data_'s alignment. Otherwise,
  // you may get lost packets. I have not been able to prevent that with attributes so far.
  uint64_t commit_time_ns_;
};

class P2PMutablePacketView {
  friend class P2PPacketView;
  template<int IC, int OC, Endianness LE> friend class P2PPacketStream;
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
  P2PPacket *packet() {
    return packet_;
  }
  P2PPacket *packet_;
};

class P2PPacketView {
  template<int IC, int OC, Endianness LE> friend class P2PPacketStream;
public:
  // Does not take ownership of the packet, which must outlive this object.
  P2PPacketView(const P2PPacket *packet) : packet_(packet) {}
  P2PPacketView(const P2PMutablePacketView &packet_view) : packet_(packet_view.packet_) {}
  P2PPacketView() : P2PPacketView(NULL) {}

  uint8_t length() const { 
    assert(packet_ != NULL);
    return packet_->length();
  }
  const uint8_t *content() const {
    assert(packet_ != NULL);
    return packet_->content();
  }
  int priority() const {
    return packet_->header()->priority;
  }
  bool is_valid() const { return packet_ != NULL; }

private:
  const P2PPacket *packet() const {
    return packet_;
  }
  const P2PPacket *packet_;
};

template<int kCapacity, Endianness LocalEndianness> class P2PPacketInputStream {
  template<int IC, int OC, Endianness LE> friend class P2PPacketStream;
public:
  // Does not take ownership of the byte stream or timer, which must outlive this object.
  // Only one packet stream can be associated to each byte stream at a time.
  P2PPacketInputStream(P2PByteStreamInterface<LocalEndianness> *byte_stream, TimerInterface *timer)
    : byte_stream_(*byte_stream), timer_(*timer), state_(kWaitingForPacket), packet_filter_(NULL) {}

  // Returns the number of times that Consume() can be called without OldestPacket() returning NULL.
  int NumAvailablePackets(P2PPriority priority) const { return packet_buffer_[priority].Size(); }

  // Returns a view to the oldest packet in the stream, or kUnavailableError if empty.
  StatusOr<const P2PPacketView> OldestPacket() { 
    P2PPacket *packet = packet_buffer_.OldestValue();
    if (packet == NULL) {
      return Status::kUnavailableError;
    }
    if (packet->commit_time_ns() != -1ULL) {
      uint64_t delay_ns = timer_.GetSystemNanoseconds() - packet->commit_time_ns();
      ++stats_.total_packets_[packet->header()->priority];
      stats_.total_packet_delay_ns_[packet->header()->priority] += delay_ns;
      stats_.total_packet_delay_per_byte_ns_[packet->header()->priority] += delay_ns / (sizeof(P2PHeader) + packet->length() + sizeof(P2PFooter));
      // Mute stats update as this function may be called multiple times for a packet.
      packet->commit_time_ns() = -1ULL;
    }
    return P2PPacketView(packet);
  }

  // Consumes the oldest packet with highest priority in the stream. Afterwards, OldestPacket() returns a new
  // value. Returns false, if there is no packet to consume.
  bool Consume(P2PPriority priority) { return packet_buffer_.Consume(priority); }

  // Not all platforms support lambdas.
  void SetPacketFilter(bool (*fn)(const P2PPacket &, void *), void *user_data) { packet_filter_ = fn; packet_filter_arg_ = user_data; }

  // Runs the stream logic. Must be called from a run loop continuously, or when there is
  // data available in the byte stream.
  void Run();

  class Stats {
    friend class P2PPacketInputStream;
  public:
    Stats() { 
      for (int i = 0; i < P2PPriority::kNumLevels; ++i) { total_packets_[i] = 0; }
      for (int i = 0; i < P2PPriority::kNumLevels; ++i) { total_packet_delay_ns_[i] = 0; }
      for (int i = 0; i < P2PPriority::kNumLevels; ++i) { total_packet_delay_per_byte_ns_[i] = 0; }
    }
    
    // Total number of packets received per priority level.
    uint64_t total_packets(P2PPriority priority) const { return total_packets_[priority]; }

    // Average delay between a packet is fully received and it is retrieved by the caller
    // with OldestPacket(). There is one value for every priority level. It is -1 if no
    // packet has been sent since the stats started.
    uint64_t average_packet_delay_ns(P2PPriority priority) const {
      return total_packets_[priority] > 0 ? total_packet_delay_ns_[priority] / total_packets_[priority] : -1;      
    }

    // The same as average_packet_delay_ns(), but with every packet delay normalized by the
    // number of packet bytes. In this way, values for packets of different lengths are
    // comparable.
    uint64_t average_packet_delay_per_byte_ns(P2PPriority priority) const {
      return total_packets_[priority] > 0 ? total_packet_delay_per_byte_ns_[priority] / total_packets_[priority] : -1;
    }

    private:
      uint64_t total_packets_[P2PPriority::kNumLevels];
      uint64_t total_packet_delay_ns_[P2PPriority::kNumLevels];
      uint64_t total_packet_delay_per_byte_ns_[P2PPriority::kNumLevels];
  };

  const Stats &stats() const { return stats_; }

private:
  PriorityRingBuffer<P2PPacket, kCapacity, P2PPriority> packet_buffer_;
  P2PByteStreamInterface<LocalEndianness> &byte_stream_;
  TimerInterface &timer_;
  unsigned int current_field_read_bytes_;
  enum State { kWaitingForPacket, kReadingHeader, kReadingContent, kDisambiguatingStartTokenInContent, kReadingFooter } state_;
  P2PHeader incoming_header_;
  bool (*packet_filter_)(const P2PPacket &, void *);
  void *packet_filter_arg_;
  
  Stats stats_;
};

template<int kCapacity, Endianness LocalEndianness> class P2PPacketOutputStream {
  template<int IC, int OC, Endianness LE> friend class P2PPacketStream;
public:
  // Does not take ownership of the byte stream or timer, which must outlive this object.
  // Only one packet stream can be associated to each byte stream at a time.
  P2PPacketOutputStream(P2PByteStreamInterface<LocalEndianness> *byte_stream, TimerInterface *timer)
    : byte_stream_(*byte_stream), timer_(*timer), current_sequence_number_(0), state_(kGettingNextPacket), packet_filter_(NULL) {}

  // Returns the number of packet slots available for writing in the stream.
  int NumAvailableSlots(P2PPriority priority) const {
    return packet_buffer_.Capacity(priority) - packet_buffer_.Size(priority);
  }

  // Returns a view to a new packet with `priority` in the stream, or kUnavailableError if no
  // space is available in the stream for the given priority. Commit() must be called for the
  // packet to be finalized.
  StatusOr<P2PMutablePacketView> NewPacket(P2PPriority priority) { 
    if (NumAvailableSlots(priority) == 0) {
      return Status::kUnavailableError;
    }
    P2PPacket &packet = packet_buffer_.NewValue(priority);
    packet.header()->is_continuation = 0;
    packet.header()->requires_ack = 0;
    packet.header()->is_ack = 0;
    packet.header()->is_init = 0;
    packet.header()->reserved = 0;
    packet.length() = 0;
    return P2PMutablePacketView(&packet);
  }

  // Commits changes to the new packet. Must be called for the packet to be sent.
  // Afterwards, previous packet views returned by NewPacket() cannot be trusted to be valid,
  // and NewPacket() returns a different view.
  bool Commit(P2PPriority priority, bool guarantee_delivery) {
    P2PPacket &packet = packet_buffer_.NewValue(priority);
    packet.header()->priority = priority;
    packet.header()->requires_ack = guarantee_delivery;
    packet.sequence_number() = current_sequence_number_;
    if (!packet.PrepareToSend()) { return false; }
    // Fix endianness.
    packet.checksum() = LocalToNetwork<LocalEndianness>(packet.checksum());
    packet.length() = LocalToNetwork<LocalEndianness>(packet.length());

    packet.commit_time_ns() = timer_.GetSystemNanoseconds();
    packet_buffer_.Commit(priority);

    ++current_sequence_number_;

    return true;
  }

  // Not all platforms support lambdas.
  void SetPacketFilter(bool (*fn)(const P2PPacket &, void *), void *user_data) { packet_filter_ = fn; packet_filter_arg_ = user_data; }

  // Runs the stream and returns the minimum number of microseconds the caller may wait
  // until calling Run() again. Multi-threaded platforms can use this value to yield time
  // to other threads.
  // If just_sent_packet_view is not NULL, it is filled with a view to the packet that Run()
  // just finished sending. The view is invalid, if Run() did not finish sending any packet
  // within the call.
  uint64_t Run();

  class Stats {
    friend class P2PPacketOutputStream;
  public:
    Stats() { 
      for (int i = 0; i < P2PPriority::kNumLevels; ++i) { total_packets_[i] = 0; }
      for (int i = 0; i < P2PPriority::kNumLevels; ++i) { total_packet_delay_ns_[i] = 0; }
      for (int i = 0; i < P2PPriority::kNumLevels; ++i) { total_packet_delay_per_byte_ns_[i] = 0; }
    }
    
    // Total number of sent packets per priority level.
    uint64_t total_packets(P2PPriority priority) const { return total_packets_[priority]; }

    // Average delay between a packet is committed and its last byte gets in the platform's
    // byte stream. There is one value for every priority level. It is -1 if no packet has
    // been sent since the stats started.
    uint64_t average_packet_delay_ns(P2PPriority priority) const {
      return total_packets_[priority] > 0 ? total_packet_delay_ns_[priority] / total_packets_[priority] : -1;      
    }

    // The same as average_packet_delay_ns(), but with every packet delay normalized by the
    // number of packet bytes. In this way, values for packets of different lengths are
    // comparable.
    uint64_t average_packet_delay_per_byte_ns(P2PPriority priority) const {
      return total_packets_[priority] > 0 ? total_packet_delay_per_byte_ns_[priority] / total_packets_[priority] : -1;
    }

    private:
      uint64_t total_packets_[P2PPriority::kNumLevels];
      uint64_t total_packet_delay_ns_[P2PPriority::kNumLevels];
      uint64_t total_packet_delay_per_byte_ns_[P2PPriority::kNumLevels];
  };

  const Stats &stats() const { return stats_; }

private:
  PriorityRingBuffer<P2PPacket, kCapacity, P2PPriority> packet_buffer_;
  P2PByteStreamInterface<LocalEndianness> &byte_stream_;
  TimerInterface &timer_;
  P2PPacket *current_packet_;
  int total_packet_bytes_[P2PPriority::kNumLevels];
  int pending_packet_bytes_;
  int total_burst_bytes_;
  int pending_burst_bytes_;  
  uint64_t after_burst_wait_end_timestamp_ns_;
  uint64_t current_sequence_number_; 
  enum State { kGettingNextPacket, kSendingHeaderBurst, kWaitingForHeaderBurstIngestion, kSendingBurst, kWaitingForBurstIngestion, kWaitingForPartialBurstIngestionBeforeHigherPriorityPacket } state_;  
  bool (*packet_filter_)(const P2PPacket &, void *);
  void *packet_filter_arg_;

  Stats stats_;
};

template<int kInputCapacity, int kOutputCapacity, Endianness LocalEndianness> class P2PPacketStream {
public:
  // Does not take ownership of the streams, which must outlive this object.
  P2PPacketStream(P2PByteStreamInterface<LocalEndianness> *byte_stream, TimerInterface *timer)
    : input_(byte_stream, timer), output_(byte_stream, timer), last_rx_sequence_number_(-1ULL) {
      input_.SetPacketFilter(&ShouldCommitInputPacket, this);
      output_.SetPacketFilter(&ShouldConsumeOutputPacket, this);
    }

  P2PPacketInputStream<kInputCapacity, LocalEndianness> &input() { return input_; }
  P2PPacketOutputStream<kOutputCapacity, LocalEndianness> &output() { return output_; }

protected:

  static bool ShouldCommitInputPacket(const P2PPacket &last_rx_packet, void *self_ptr) {
    P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness> &self = *reinterpret_cast<P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness> *>(self_ptr);
    if (last_rx_packet.header()->is_ack) {
      // We got an ACK: discard the retrainsmitting packet that originated it.

      // ACKs always have a priority one level higher to avoid deadlocks. Turn priority down one
      // notch to get that of the retransmitting packet.
      P2PPriority data_packet_priority = last_rx_packet.header()->priority + 1;

      const P2PPacket &retransmitting_packet = *self.output_.packet_buffer_.OldestValue(data_packet_priority);
      // Check the retransmitting packet exists, as it could have been consumed already by a
      // previous ACK.
      if (retransmitting_packet.header()->requires_ack &&
          last_rx_packet.sequence_number() == retransmitting_packet.sequence_number()) {
        self.output_.packet_buffer_.Consume(data_packet_priority);
      }

      // Do not expose an ACK in the API.
      return false;
    }

    // It's a data packet: reply with ACK if there is no ACK in the output buffer already,
    // to avoid flooding the buffer and blocking the sender for this priority and lower.
    if (last_rx_packet.header()->requires_ack) {
      // ACKs always have a priority one level higher to avoid deadlocks.
      P2PPriority ack_priority = last_rx_packet.header()->priority - 1;

      bool ack_found = false;
      for (int i = 0; i < self.output_.packet_buffer_.Size(ack_priority); ++i) {
        const P2PPacket *maybe_ack_packet = self.output_.packet_buffer_.OldestValue(ack_priority, i);
        assert(maybe_ack_packet != NULL);
        if (maybe_ack_packet->sequence_number() == last_rx_packet.sequence_number()) {
          ack_found = true;
          break;
        }
      }
      if (!ack_found) {
        StatusOr<P2PMutablePacketView> ack_packet_view = self.output_.NewPacket(ack_priority);
        if (!ack_packet_view.ok()) {
          // Only commit the packet if there is space for the ACK in the output stream.
          // Let the other end keep retransmitting until there's space for the ACK.
          return false;
        }
        P2PPacket *ack = ack_packet_view->packet();
        ack->header()->is_ack = 1;
        self.output_.Commit(ack_priority, /*guaranteed_delivery=*/false);
      }

      if (self.last_rx_sequence_number_ != -1ULL && last_rx_packet.sequence_number() <= self.last_rx_sequence_number_) {
        // This packet had been received already: filter it.
        return false;
      }
      self.last_rx_sequence_number_ = last_rx_packet.sequence_number();
    }
    
    // Expose the packet in the API.
    return true;
  }

  static bool ShouldConsumeOutputPacket(const P2PPacket &last_tx_packet, void *self_ptr) {
    // Packets requiring an ACK are left in the queue for retransmission.
    return !last_tx_packet.header()->requires_ack;
  }

private:
  P2PPacketInputStream<kInputCapacity, LocalEndianness> input_;
  P2PPacketOutputStream<kOutputCapacity, LocalEndianness> output_;
  uint64_t last_rx_sequence_number_;
};

#include "p2p_packet_stream.hh"

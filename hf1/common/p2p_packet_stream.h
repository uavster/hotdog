// A buffered stream of P2P packets.
// 
// The Each pair of 

#ifndef P2P_PACKET_STREAM_
#define P2P_PACKET_STREAM_

#include "p2p_packet_protocol.h"
#include "p2p_byte_stream_interface.h"
#include "priority_ring_buffer.h"
#include "status_or.h"
#include "timer_interface.h"
#include "guid_factory_interface.h"
#include "logger_interface.h"

// Represents the priority of a packet.
// The higher the priority, the lower the latency, as higher priority packets
// preempt lower priority ones in both the transmitter and receiver.
class P2PPriority {
public:
  enum Level { kReserved = 0, kHigh, kMedium, kLow, kNumLevels };

  P2PPriority(Level level) : level_(level) {}  

  P2PPriority(int numeric_priority) : level_(static_cast<Level>(numeric_priority)) {
    ASSERT(level_ < kNumLevels);
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

// A maximum-length P2P packet with convenience accessors.
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

  bool &counted_in_stats() { return counted_in_stats_; };
  bool counted_in_stats() const { return counted_in_stats_; };

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
  // you may get lost packets. I have not been able to prevent that with compiler attributes so far.
  uint64_t commit_time_ns_;
  bool counted_in_stats_;
};

// A mutable view to a packet's content.
// Mutating the view is mutating the associated packet.
class P2PMutablePacketView {
  friend class P2PPacketView;
  template<int IC, int OC, Endianness LE> friend class P2PPacketStream;
public:
  // Does not take ownership of the packet, which must outlive this object.
  P2PMutablePacketView(P2PPacket *packet) : packet_(packet) {}
  P2PMutablePacketView() : P2PMutablePacketView(NULL) {}

  uint8_t length() const { 
    ASSERT(packet_ != NULL);
    return packet_->length();
  }
  uint8_t &length() { 
    ASSERT(packet_ != NULL);
    return packet_->length();
  }

  const uint8_t *content() const { 
    ASSERT(packet_ != NULL);
    return packet_->content();
  }
  uint8_t *content() { 
    ASSERT(packet_ != NULL);
    return packet_->content();
  }

  int priority() const {
    ASSERT(packet_ != NULL);
    return packet_->header()->priority;
  }

private:
  P2PPacket *packet() {
    return packet_;
  }
  P2PPacket *packet_;
};

// An immutable view to a packet.
class P2PPacketView {
  template<int IC, int OC, Endianness LE> friend class P2PPacketStream;
public:
  // Does not take ownership of the packet, which must outlive this object.
  P2PPacketView(const P2PPacket *packet) : packet_(packet) {}
  P2PPacketView(const P2PMutablePacketView &packet_view) : packet_(packet_view.packet_) {}
  P2PPacketView() : P2PPacketView(NULL) {}

  uint8_t length() const { 
    ASSERT(packet_ != NULL);
    return packet_->length();
  }
  const uint8_t *content() const {
    ASSERT(packet_ != NULL);
    return packet_->content();
  }
  int priority() const {
    ASSERT(packet_ != NULL);
    return packet_->header()->priority;
  }
  bool is_valid() const { return packet_ != NULL; }

  uint64_t reception_local_time_ns() const {
    ASSERT(packet_ != NULL);
    return packet_->commit_time_ns();
  }

private:
  const P2PPacket *packet() const {
    return packet_;
  }
  const P2PPacket *packet_;
};

template<typename TFunctionPtr, typename TArg> class P2PCallback {
public:
  P2PCallback() : function_(nullptr) {}
  P2PCallback(TFunctionPtr fn, TArg user_data) 
    : function_(fn), arg_(user_data) {
  }
  
  TFunctionPtr function() const { return function_; }
  TArg arg() const { return arg_; }  

private:
  TFunctionPtr function_;
  TArg arg_;
};

class P2PPacketFilter : public P2PCallback<bool (*)(const P2PPacket &, void *), void *> {
public:
  P2PPacketFilter() : P2PCallback<bool (*)(const P2PPacket &, void *), void *>() {}
  P2PPacketFilter(bool (*fn)(const P2PPacket &, void *), void *args) 
    : P2PCallback<bool (*)(const P2PPacket &, void *), void *>(fn, args) {}

  bool operator()(const P2PPacket &p) {
    if (function() == NULL) {
      return true;
    }
    return function()(p, arg());
  }
};

class P2PPacketCommittedCallback : public P2PCallback<void (*)(const P2PPacket &, void *), void *> {
public:
  P2PPacketCommittedCallback() : P2PCallback<void (*)(const P2PPacket &, void *), void *>() {}
  P2PPacketCommittedCallback(void (*fn)(const P2PPacket &, void *), void *args) 
    : P2PCallback<void (*)(const P2PPacket &, void *), void *>(fn, args) {}

  void operator()(const P2PPacket &p) {
    if (function() != NULL) {
      function()(p, arg());
    }
  }
};

// Represents a buffered input stream of best-effort packets with priorities. 
// Higher-priority packets are received and delivered to the caller earlier than lower-priority 
// ones thanks to a preemption and continuation mechanism.
// Packets of any given priority can be lost (overwritten by newer packets, starting by the oldest)
// if the caller cannot consume the packets fast enough.
// This stream does not send any information to the other end. Any signaling to the other end
// must be handled by the caller with an output stream.
template<int kCapacity, Endianness LocalEndianness> class P2PPacketInputStream {
  template<int IC, int OC, Endianness LE> friend class P2PPacketStream;
public:
  // Does not take ownership of the byte stream or timer, which must outlive this object.
  // Only one packet stream can be associated to each byte stream at a time.
  P2PPacketInputStream(P2PByteStreamInterface<LocalEndianness> *byte_stream, TimerInterface *timer)
    : byte_stream_(*byte_stream), timer_(*timer) {
      Reset();
    }

  // Clears the packet buffers and resets the state machine.
  void Reset();

  // Returns the number of times that Consume() can be called without OldestPacket() returning NULL.
  int NumAvailablePackets(P2PPriority priority) const { return packet_buffer_[priority].Size(); }

  // Returns a view to the oldest packet in the stream, or kUnavailableError if empty.
  StatusOr<const P2PPacketView> OldestPacket();

  // Consumes the oldest packet with highest priority in the stream. Afterwards, OldestPacket() returns a new
  // value. Returns false, if there is no packet to consume.
  bool Consume(P2PPriority priority) { return packet_buffer_.Consume(priority); }

  // Not all platforms support lambdas.
  void packet_filter(const P2PPacketFilter &filter) { packet_filter_ = filter; }
  const P2PPacketFilter &packet_filter() { return packet_filter_; }

  // Runs the stream logic. Must be called from a run loop continuously, or when there is
  // data available in the byte stream. Returns the number of bytes read and processed.
  int Run();

  // Reception statistics.
  class Stats {
    friend class P2PPacketInputStream;
  public:
    Stats() { 
      for (int i = 0; i < P2PPriority::kNumLevels; ++i) { 
        total_packets_[i] = 0;
        total_packet_delay_ns_[i] = 0;
        total_packet_delay_per_byte_ns_[i] = 0;
      }
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
  P2PPacket *incoming_packet_[P2PPriority::kNumLevels];
  P2PPacketFilter packet_filter_;
  uint8_t write_offset_before_break_[P2PPriority::kNumLevels];
  uint64_t current_sequence_number_[P2PPriority::kNumLevels];
  P2PPacket discarded_packet_placeholder_;

  Stats stats_;
};

// Represents a buffered output stream of best-effort packets. 
// Higher-priority packets are sent earlier over the link than lower-priority ones
// thanks to a preemption and continuation mechanism.
// This stream does not receive any information from the other end.
// Any signaling from the other end must be handled by the caller with an input stream.
template<int kCapacity, Endianness LocalEndianness> class P2PPacketOutputStream {
  template<int IC, int OC, Endianness LE> friend class P2PPacketStream;
public:
  // Does not take ownership of the byte stream or timer, which must outlive this object.
  // Only one packet stream can be associated to each byte stream at a time.
  P2PPacketOutputStream(P2PByteStreamInterface<LocalEndianness> *byte_stream, TimerInterface *timer)
    : byte_stream_(*byte_stream), timer_(*timer) {
      Reset();
    }

  // Clears the packet buffers and resets the state machine.
  void Reset();

  // Returns the number of packet slot available for writing.
  int NumAvailableSlots(P2PPriority priority) const {
    return packet_buffer_.NumAvailableSlots(priority);
  }

  // Returns the number of committed packets waiting to be sent for a given priority.
  int NumCommittedPackets(P2PPriority priority) const {
    return packet_buffer_.Size(priority);
  }
  // Returns the number of committed packets waiting to be sent for all priorities.
  int NumCommittedPackets() const;    

  // Returns a view to a new packet with `priority` in the stream, or kUnavailableError if no
  // space is available in the stream for the given priority. Commit() must be called with the
  // for the same `priority` for the packet to be finalized.
  StatusOr<P2PMutablePacketView> NewPacket(P2PPriority priority);

  // Commits changes to the new packet. Must be called for the packet to be sent.
  // Afterwards, previous packet views returned by NewPacket() cannot be trusted to be valid,
  // and NewPacket() returns a different view.
  // If `guarantee_delivery` is true, the packet will be retransmitted until the other end 
  // acknowledges its reception. Use it with care because, in the meantime, the transmission of other 
  // output packets with the same or lower priority will be put on hold. Take this especially into 
  // account if a long disruption in the other end's reception is expected (e.g. a delay in calling
  // the communication handling code or a link disconnection). In that case, the lower or equal
  // priority levels in the output queue could quickly fill up if there are processeses transmitting
  // periodically.
  bool Commit(P2PPriority priority, bool guarantee_delivery, uint64_t seq_number = -1ULL);

  P2PPacketCommittedCallback packet_committed_callback() const { return packet_committed_callback_; }
  void packet_committed_callback(const P2PPacketCommittedCallback &callback) { packet_committed_callback_ = callback; }

  // Not all platforms support lambdas.
  void packet_filter(const P2PPacketFilter &filter) { packet_filter_ = filter; }
  const P2PPacketFilter &packet_filter() { return packet_filter_; }

  // Runs the stream and returns the minimum number of microseconds the caller may wait
  // until calling Run() again. Multi-threaded platforms can use this value to yield time
  // to other threads.
  uint64_t Run();

  // Transmission statistics.
  class Stats {
    friend class P2PPacketOutputStream;
  public:
    Stats() { 
      for (int i = 0; i < P2PPriority::kNumLevels; ++i) { total_packets_[i] = 0; }
      for (int i = 0; i < P2PPriority::kNumLevels; ++i) { total_reliable_packets_[i] = 0; }
      for (int i = 0; i < P2PPriority::kNumLevels; ++i) { total_packet_delay_ns_[i] = 0; }
      for (int i = 0; i < P2PPriority::kNumLevels; ++i) { total_packet_delay_per_byte_ns_[i] = 0; }
      for (int i = 0; i < P2PPriority::kNumLevels; ++i) { total_retransmissions_[i] = 0; }
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

    float average_retransmissions_per_reliable_packet(P2PPriority priority) const {
      return total_retransmissions_[priority] / static_cast<float>(total_reliable_packets_[priority]);
    }

    private:
      uint64_t total_packets_[P2PPriority::kNumLevels];
      uint64_t total_reliable_packets_[P2PPriority::kNumLevels];
      uint64_t total_packet_delay_ns_[P2PPriority::kNumLevels];
      uint64_t total_packet_delay_per_byte_ns_[P2PPriority::kNumLevels];
      uint64_t total_retransmissions_[P2PPriority::kNumLevels];
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
  uint64_t current_sequence_number_[P2PPriority::kNumLevels];
  uint64_t last_sent_sequence_number_[P2PPriority::kNumLevels];
  enum State { kGettingNextPacket, kSendingHeaderBurst, kWaitingForHeaderBurstIngestion, kSendingBurst, kWaitingForBurstIngestion, kWaitingForPartialBurstIngestionBeforeHigherPriorityPacket } state_;  
  P2PPacketFilter packet_filter_;
  P2PPacketCommittedCallback packet_committed_callback_;

  Stats stats_;
};

class P2POtherEndStartedCallback : public P2PCallback<void (*)(void *), void *> {
public:
  P2POtherEndStartedCallback() : P2PCallback<void (*)(void *), void *>() {}
  P2POtherEndStartedCallback(void (*fn)(void *), void *args) 
    : P2PCallback<void (*)(void *), void *>(fn, args) {}

  void operator()() {
    if (function() != NULL) {
      function()(arg());
    }
  }
};

// A buffered input/output stream of packets with priorities. 
// The caller can choose to send packets reliably or as best effort. 
// Reliable packets are retransmitted until the other end acknowledges them.
template<int kInputCapacity, int kOutputCapacity, Endianness LocalEndianness> class P2PPacketStream {
public:
  // Does not take ownership of the streams, which must outlive this object.
  P2PPacketStream(P2PByteStreamInterface<LocalEndianness> *byte_stream, TimerInterface *timer, GUIDFactoryInterface &guid_factory);

  P2PPacketInputStream<kInputCapacity, LocalEndianness> &input() { return input_; }
  P2PPacketOutputStream<kOutputCapacity, LocalEndianness> &output() { return output_; }

  // Sets a callback that's called when the other end sends an init packet. The callback is 
  // used to ensure that all the state previously created in the other end is recreated. 
  // The function pointed by the callback object must outlive this stream.
  void other_end_started_callback(const P2POtherEndStartedCallback &callback) {
    other_end_started_callback_ = callback;
  }
  P2POtherEndStartedCallback other_end_started_callback() const { 
    return other_end_started_callback_;
  }

protected:
  void ResetInput();
  void ResetOutputSession(const P2PPacket &handshake_request);

  // Returns true if the acknowledge for the passed `packet` was already committed to the
  // output stream.
  bool IsACKCommittedForPacket(const P2PPacket &packet);

  // Returns false if the ACK packet was to be scheduled, but there was not space in the output
  // buffer. Returns true if no new ACK was required, or if it was scheduled successfully,
  // otherwise.
  bool ScheduleACKWithThrottling(const P2PPacket &packet);

  static bool ShouldCommitInputPacket(const P2PPacket &last_rx_packet, void *self_ptr);
  static bool ShouldConsumeOutputPacket(const P2PPacket &last_tx_packet, void *self_ptr);

private:
  P2PPacketInputStream<kInputCapacity, LocalEndianness> input_;
  P2PPacketOutputStream<kOutputCapacity, LocalEndianness> output_;
  P2PSequenceNumberType handshake_id_;
  bool handshake_done_;
  uint64_t last_rx_sequence_number_[P2PPriority::kNumLevels];
  uint64_t last_init_sequence_number_[P2PPriority::kNumLevels];
  P2POtherEndStartedCallback other_end_started_callback_;
};

#include "p2p_packet_stream.hh"

#endif  // P2P_PACKET_STREAM_
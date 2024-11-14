#include <algorithm>

template<int kCapacity, Endianness LocalEndianness>
void P2PPacketInputStream<kCapacity, LocalEndianness>::Reset() {
  packet_buffer_.Clear();
  for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
    write_offset_before_break_[i] = 0;
    incoming_packet_[i] = nullptr;
  }
  state_ = kWaitingForPacket;
}

template<int kCapacity, Endianness LocalEndianness>
StatusOr<const P2PPacketView> P2PPacketInputStream<kCapacity, LocalEndianness>::OldestPacket() { 
  P2PPacket *packet = packet_buffer_.OldestValue();
  if (packet == NULL) {
    return Status::kUnavailableError;
  }
  if (!packet->counted_in_stats()) {
    uint64_t delay_ns = timer_.GetLocalNanoseconds() - packet->commit_time_ns();
    ++stats_.total_packets_[packet->header()->priority];
    stats_.total_packet_delay_ns_[packet->header()->priority] += delay_ns;
    stats_.total_packet_delay_per_byte_ns_[packet->header()->priority] += delay_ns / (sizeof(P2PHeader) + packet->length() + sizeof(P2PFooter));
    // Mute stats update as this function may be called multiple times for a packet.
    packet->counted_in_stats() = true;
  }
  return P2PPacketView(packet);
}

template<int kCapacity, Endianness LocalEndianness>
void P2PPacketOutputStream<kCapacity, LocalEndianness>::Reset() {
  packet_buffer_.Clear();
  for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
    current_sequence_number_[i] = 0;
    last_sent_sequence_number_[i] = -1ULL;
    total_packet_bytes_[i] = -1;
  }
  state_ = kGettingNextPacket;
}

template<int kCapacity, Endianness LocalEndianness>
StatusOr<P2PMutablePacketView> P2PPacketOutputStream<kCapacity, LocalEndianness>::NewPacket(P2PPriority priority) {
  if (packet_buffer_.IsFull(priority)) {
    return Status::kUnavailableError;
  }
  P2PPacket &packet = packet_buffer_.NewValue(priority);
  packet.header()->priority = priority;
  packet.header()->is_continuation = 0;
  packet.header()->requires_ack = 0;
  packet.header()->is_ack = 0;
  packet.header()->is_init = 0;
  packet.header()->reserved = 0;
  packet.length() = 0;
  return P2PMutablePacketView(&packet);
}

template<int kCapacity, Endianness LocalEndianness>
bool P2PPacketOutputStream<kCapacity, LocalEndianness>::Commit(P2PPriority priority, bool guarantee_delivery, uint64_t seq_number) {
  P2PPacket &packet = packet_buffer_.NewValue(priority);
  packet.header()->priority = priority;
  packet.header()->requires_ack = guarantee_delivery;
  if (seq_number == -1ULL) {
    packet.sequence_number() = current_sequence_number_[priority];
  } else {
    packet.sequence_number() = seq_number;
  }
  if (!packet.PrepareToSend()) { return false; }
  // Fix endianness.
  packet.checksum() = LocalToNetwork<LocalEndianness>(packet.checksum());
  packet.length() = LocalToNetwork<LocalEndianness>(packet.length());

  packet.counted_in_stats() = false;
  packet.commit_time_ns() = timer_.GetLocalNanoseconds();
  packet_buffer_.Commit(priority);

  if (seq_number == -1ULL) {
    ++current_sequence_number_[priority];
  }

  packet_committed_callback_(packet);

  return true;
}

template<int kCapacity, Endianness LocalEndianness> int P2PPacketInputStream<kCapacity, LocalEndianness>::Run() {
  int num_bytes_read = 0;
  switch (state_) {
    case kWaitingForPacket:
      {
        if ((num_bytes_read = byte_stream_.Read(&incoming_header_.start_token, 1)) < 1) {
          break;
        }
        if (incoming_header_.start_token == kP2PStartToken) {
          state_ = kReadingHeader;
          current_field_read_bytes_ = 1;
        }
        break;
      }

    case kReadingHeader:
      {
        if (current_field_read_bytes_ >= sizeof(P2PHeader)) {
          // If it's a new packet, put the received header in a new slot at the given
          // priority. If it's a continuation of a previous packet, the header is already
          // there.
          if (incoming_header_.priority >= P2PPriority::kNumLevels) {
            // Invalid priority level.
            state_ = kWaitingForPacket;
            break;
          }

          if (!incoming_header_.is_continuation) {
            // New packet.

            if (!packet_buffer_.IsFull(incoming_header_.priority)) {
              // There is buffer space: get the next empty slot.
              incoming_packet_[incoming_header_.priority] = &packet_buffer_.NewValue(incoming_header_.priority);
            } else {
              // No space available.
              if (!incoming_header_.requires_ack) {
                // It's a regular packet: finish receiving it without writing it in the 
                // input queue.
                incoming_packet_[incoming_header_.priority] = &discarded_packet_placeholder_;
              } else {
                // It's a guaranteed-delivery packet: discard one regular packet to make room.
                for (int i = 0; i < packet_buffer_.Size(incoming_header_.priority); ++i) {
                  if (!packet_buffer_.OldestValue(incoming_header_.priority, i)->header()->requires_ack) {
                    packet_buffer_.Consume(incoming_header_.priority, i);
                    break;
                  }
                }
                if (!packet_buffer_.IsFull(incoming_header_.priority)) {
                  // We were able to discard one regular packet: use the new room for the 
                  // guaranteed-delivery packet.
                  incoming_packet_[incoming_header_.priority] = &packet_buffer_.NewValue(incoming_header_.priority);
                } else {
                  // There were no regular packets to discard: finish reading the packet, but
                  // store it in bogus location to still react to inconsistencies. The other
                  // end should keep resending it until we have input buffer space to receive it.
                  incoming_packet_[incoming_header_.priority] = &discarded_packet_placeholder_;
                }
              }              
            }

            P2PPacket &packet = *incoming_packet_[incoming_header_.priority];
            for (unsigned int i = 0; i < sizeof(P2PHeader); ++i) {
              reinterpret_cast<uint8_t *>(&packet)[i] = reinterpret_cast<uint8_t *>(&incoming_header_)[i];
            }
            // Fix endianness of header fields, so they can be used locally in next states.
            packet.length() = NetworkToLocal<LocalEndianness>(packet.length());
            write_offset_before_break_[incoming_header_.priority] = 0;
            current_field_read_bytes_ = 0;
          } else {
            // Continuing a packet previously interrupted by a higher-priority packet.

            ASSERT(incoming_packet_[incoming_header_.priority] != nullptr);
            P2PPacket &packet = *incoming_packet_[incoming_header_.priority];
            // The length field for a packet continuation is the remaining length.
            int remaining_length = NetworkToLocal<LocalEndianness>(incoming_header_.length);
            if (incoming_header_.sequence_number != packet.sequence_number() || 
                remaining_length != packet.length() - write_offset_before_break_[incoming_header_.priority]) {
              // This continuation does not belong to the packet we have in store, or the
              // continuation offset is not where we left off (could be a continuation from a
              // different retransmission). There must have been a link interruption: reset the
              // state machine.
              state_ = kWaitingForPacket;
              break;
            }
            // Keep receiving content where we left off.
            current_field_read_bytes_ = packet.length() - remaining_length;
          }
          state_ = kReadingContent;
          break;
        }
        uint8_t *current_byte = &reinterpret_cast<uint8_t *>(&incoming_header_)[current_field_read_bytes_];
        num_bytes_read = byte_stream_.Read(current_byte, 1);
        if (num_bytes_read < 1) {
          break;
        }
        current_field_read_bytes_ += num_bytes_read;
        if (*current_byte == kP2PStartToken) {
          // Must be a new packet after a link interruption because priority takeover is
          // not legal mid-header.
          state_ = kReadingHeader;
          incoming_header_.start_token = kP2PStartToken;
          current_field_read_bytes_ = 1;
          break;
        }
        if (*current_byte == kP2PSpecialToken) {
          // Malformed packet.
          state_ = kWaitingForPacket;
        }
        break;
      }

    case kReadingContent:
      {
        ASSERT(incoming_packet_[incoming_header_.priority] != nullptr);
        P2PPacket &packet = *incoming_packet_[incoming_header_.priority];
        if (current_field_read_bytes_ >= packet.length()) {
          state_ = kReadingFooter;
          current_field_read_bytes_ = 0;
          break;
        }
        uint8_t *next_content_byte = &packet.content()[current_field_read_bytes_];
        num_bytes_read = byte_stream_.Read(next_content_byte, 1);
        if (num_bytes_read < 1) {
          break;
        }
        current_field_read_bytes_ += num_bytes_read;
        if (*next_content_byte == kP2PStartToken) {
          if (current_field_read_bytes_ < packet.length()) {
            // It could be a start token, if the next byte is not a special token.
            state_ = kDisambiguatingStartTokenInContent;
          } else {
            // If there can't be a special token next because this is the last content byte,
            // it could either be a malformed packet or a new packet start. Assume the other
            // end will form correct packets. The start token may then be due to a new packet
            // after a link interruption, or a packet with higher priority.
            write_offset_before_break_[incoming_header_.priority] = current_field_read_bytes_ - 1;
            state_ = kReadingHeader;
            incoming_header_.start_token = kP2PStartToken;
            current_field_read_bytes_ = 1;
          }
        }
        break;
      }

    case kDisambiguatingStartTokenInContent:
      {
        ASSERT(incoming_packet_[incoming_header_.priority] != nullptr);
        P2PPacket &packet = *incoming_packet_[incoming_header_.priority];
        // Read next byte and check if it's a special token.
        // No need to check if we reached the content length here, as a content byte matching the start token should always be followed by a special token.
        uint8_t *next_content_byte = &packet.content()[current_field_read_bytes_];
        num_bytes_read = byte_stream_.Read(next_content_byte, 1);
        if (num_bytes_read < 1) {
          break;
        }
        current_field_read_bytes_ += num_bytes_read;
        if (*next_content_byte == kP2PSpecialToken) {
          // Not a start token, but a content byte.
          state_ = kReadingContent;
        } else {
          if (*next_content_byte == kP2PStartToken) {
            // Either a malformed packet, or a new packet after link reestablished, or a
            // higher priority packet.
            // Assume well designed transmitter and try the latter.
            write_offset_before_break_[incoming_header_.priority] = current_field_read_bytes_ - 1;
            state_ = kReadingHeader;
            incoming_header_.start_token = kP2PStartToken;
            current_field_read_bytes_ = 1;
          } else {
            // Must be a start token. Restart the state to re-synchronize with minimal latency.
            write_offset_before_break_[incoming_header_.priority] = current_field_read_bytes_ - 2;
            state_ = kReadingHeader;
            incoming_header_.start_token = kP2PStartToken;
            reinterpret_cast<uint8_t *>(&incoming_header_)[1] = *next_content_byte;
            current_field_read_bytes_ = 2;
          }
        }
      }
      break;

    case kReadingFooter:
      {
        ASSERT(incoming_packet_[incoming_header_.priority] != nullptr);
        P2PPacket &packet = *incoming_packet_[incoming_header_.priority];
        if (current_field_read_bytes_ >= sizeof(P2PFooter)) {
          state_ = kWaitingForPacket;
          break;
        }
        uint8_t *current_byte = packet.content() + packet.length() + current_field_read_bytes_;
        num_bytes_read = byte_stream_.Read(current_byte, 1);
        if (num_bytes_read < 1) {
          break;
        }
        current_field_read_bytes_ += num_bytes_read;

        if (*current_byte == kP2PStartToken) {
          // New packet after interrupts, as no priority takeover is allowed mid-footer.
          write_offset_before_break_[incoming_header_.priority] = packet.length();
          state_ = kReadingHeader;
          incoming_header_.start_token = kP2PStartToken;
          current_field_read_bytes_ = 1;
          break;
        }
        if (*current_byte == kP2PSpecialToken) {
          // Malformed packet.
          state_ = kWaitingForPacket;
        }

        if (current_field_read_bytes_ >= sizeof(P2PFooter)) {
          // Adapt endianness of footer fields.
          packet.checksum() = NetworkToLocal<LocalEndianness>(packet.checksum());
          if (packet.PrepareToRead()) {
            if (packet_filter_(packet)) {
              packet.counted_in_stats() = false;
              packet.commit_time_ns() = timer_.GetLocalNanoseconds();
              packet_buffer_.Commit(incoming_header_.priority);
            }
          }
          state_ = kWaitingForPacket;
        }
      }
      break;
  }
  return num_bytes_read;
}

template<int kCapacity, Endianness LocalEndianness> uint64_t P2PPacketOutputStream<kCapacity, LocalEndianness>::Run() {
  uint64_t time_until_next_event = 0;
  switch (state_) {
    case kGettingNextPacket:
      {
        current_packet_ = packet_buffer_.OldestValue();
        if (current_packet_ == NULL) {
          // No more packets to send: keep waiting for one.
          break;
        }

        // Start sending the new packet.
        P2PPriority priority = current_packet_->header()->priority;
        if (!current_packet_->header()->is_continuation) {
          // Full packet length.
          total_packet_bytes_[priority] = sizeof(P2PHeader) + NetworkToLocal<LocalEndianness>(current_packet_->length()) + sizeof(P2PFooter);
        } else {
          // Continuation without a previous original packet is invalid.
          ASSERT(total_packet_bytes_[priority] >= 0);
        }
        pending_packet_bytes_ = sizeof(P2PHeader);

        state_ = kSendingHeaderBurst;
        total_burst_bytes_ = std::min(pending_packet_bytes_, byte_stream_.GetBurstMaxLength());
        pending_burst_bytes_ = total_burst_bytes_;
        break;
      }

    case kSendingHeaderBurst: {
      const int written_bytes = byte_stream_.Write(
        &reinterpret_cast<const uint8_t *>(current_packet_->header())[sizeof(P2PHeader) - pending_packet_bytes_],
        pending_burst_bytes_);
      pending_packet_bytes_ -= written_bytes;
      pending_burst_bytes_ -= written_bytes;

      const uint64_t timestamp_ns = timer_.GetLocalNanoseconds();
      if (pending_packet_bytes_ <= 0) { 
        after_burst_wait_end_timestamp_ns_ = timestamp_ns + total_burst_bytes_ * byte_stream_.GetBurstIngestionNanosecondsPerByte();
        state_ = kWaitingForHeaderBurstIngestion;
        break;
      }

      if (pending_burst_bytes_ <= 0) {
        // Burst fully sent: calculate when to start the next burst.
        after_burst_wait_end_timestamp_ns_ = timestamp_ns + total_burst_bytes_ * byte_stream_.GetBurstIngestionNanosecondsPerByte();
        state_ = kWaitingForHeaderBurstIngestion;
        break;
      }

      break;
    }

    case kWaitingForHeaderBurstIngestion:
      {
        const uint64_t timestamp_ns = timer_.GetLocalNanoseconds();
        if (timestamp_ns < after_burst_wait_end_timestamp_ns_) {
          // Ingestion time not expired: keep waiting.
          time_until_next_event = after_burst_wait_end_timestamp_ns_ - timestamp_ns;
          break;
        }

        if (pending_packet_bytes_ <= 0) { 
          // Header was fully sent just now: adjust the pending bytes: adjust the pending bytes.
          if (current_packet_->header()->is_continuation) {
            pending_packet_bytes_ = NetworkToLocal<LocalEndianness>(current_packet_->length()) + sizeof(P2PFooter);
          } else {
            pending_packet_bytes_ = total_packet_bytes_[current_packet_->header()->priority] - sizeof(P2PHeader);
          }

          state_ = kSendingBurst;
          total_burst_bytes_ = std::min(pending_packet_bytes_, byte_stream_.GetBurstMaxLength());
          pending_burst_bytes_ = total_burst_bytes_;
          break;
        }

        state_ = kSendingHeaderBurst;
        total_burst_bytes_ = std::min(pending_packet_bytes_, byte_stream_.GetBurstMaxLength());
        pending_burst_bytes_ = total_burst_bytes_;
      }

    case kSendingBurst:
      {
        P2PPriority priority = current_packet_->header()->priority;
        const int written_bytes = byte_stream_.Write(
          &reinterpret_cast<const uint8_t *>(current_packet_->header())[total_packet_bytes_[priority] - pending_packet_bytes_],
          std::min(byte_stream_.GetAtomicSendMaxLength(), pending_burst_bytes_));
        pending_packet_bytes_ -= written_bytes;
        pending_burst_bytes_ -= written_bytes;

        const uint64_t timestamp_ns = timer_.GetLocalNanoseconds();
        if (pending_packet_bytes_ <= 0) {
          if (!current_packet_->header()->is_init) {
            // is_init is filtered to avoid confusing all following packets with retransmissions,
            // as is_init packets have a random sequence number.
            if (last_sent_sequence_number_[priority] == -1ULL || 
              current_packet_->sequence_number() > last_sent_sequence_number_[priority]) {
              last_sent_sequence_number_[priority] = current_packet_->sequence_number();
              // Not a retransmission: update latency stats.
              const uint64_t packet_delay = timestamp_ns - current_packet_->commit_time_ns();
              ++stats_.total_packets_[priority];
              stats_.total_packet_delay_ns_[priority] += packet_delay;
              stats_.total_packet_delay_per_byte_ns_[priority] += packet_delay / total_packet_bytes_[priority];
              if (current_packet_->header()->requires_ack) {
                ++stats_.total_reliable_packets_[priority];
              }
            } else {
              // Update retransmission stats.
              ++stats_.total_retransmissions_[priority];
            }
          }

          if (packet_filter_(*current_packet_)) {
            packet_buffer_.Consume(current_packet_->header()->priority);
          }

          after_burst_wait_end_timestamp_ns_ = timestamp_ns + total_burst_bytes_ * byte_stream_.GetBurstIngestionNanosecondsPerByte();
          state_ = kWaitingForBurstIngestion;
          break;
        }

        if (pending_burst_bytes_ <= 0) {
          // Burst fully sent: calculate when to start the next burst.
          after_burst_wait_end_timestamp_ns_ = timestamp_ns + total_burst_bytes_ * byte_stream_.GetBurstIngestionNanosecondsPerByte();
          state_ = kWaitingForBurstIngestion;
          break;
        }

        // Header has been sent already: we can break the transfer for a higher priority
        // packet now.
        const P2PPacket *maybe_higher_priority_packet = packet_buffer_.OldestValue();
        if (maybe_higher_priority_packet != NULL && maybe_higher_priority_packet != current_packet_) {
          // There is a higher priority packet waiting: mark the current one as needing
          // continuation.
          current_packet_->header()->is_continuation = 1;
          current_packet_->length() = LocalToNetwork<LocalEndianness>(pending_packet_bytes_ - sizeof(P2PFooter));
          after_burst_wait_end_timestamp_ns_ = timestamp_ns + (total_burst_bytes_ - pending_burst_bytes_) * byte_stream_.GetBurstIngestionNanosecondsPerByte();
          state_ = kWaitingForPartialBurstIngestionBeforeHigherPriorityPacket;
        }

        break;
      }

    case kWaitingForBurstIngestion:
      {
        const uint64_t timestamp_ns = timer_.GetLocalNanoseconds();
        if (timestamp_ns < after_burst_wait_end_timestamp_ns_) {
          // Ingestion time not expired: keep waiting.
          time_until_next_event = after_burst_wait_end_timestamp_ns_ - timestamp_ns;
          break;
        }

        // Burst should have been ingested by the other end.
        if (pending_packet_bytes_ <= 0) {
          // No more bursts: next packet.
          state_ = kGettingNextPacket;
          break;
        }

        state_ = kSendingBurst;
        total_burst_bytes_ = std::min(pending_packet_bytes_, byte_stream_.GetBurstMaxLength());
        pending_burst_bytes_ = total_burst_bytes_;

        break;
      }

    case kWaitingForPartialBurstIngestionBeforeHigherPriorityPacket:
      {
        const uint64_t timestamp_ns = timer_.GetLocalNanoseconds();
        if (timestamp_ns < after_burst_wait_end_timestamp_ns_) {
          // Ingestion time not expired: keep waiting.
          time_until_next_event = after_burst_wait_end_timestamp_ns_ - timestamp_ns;
          break;
        }

        state_ = kGettingNextPacket;
        break;
      }
  }
  return time_until_next_event;
}

template<int kCapacity, Endianness LocalEndianness> 
int P2PPacketOutputStream<kCapacity, LocalEndianness>::NumCommittedPackets() const {
  int num_packets = 0;
  for (int priority = 0; priority < P2PPriority::kNumLevels; ++priority) {
    num_packets += packet_buffer_.Size(priority);
  }
  return num_packets;
}

template<int kInputCapacity, int kOutputCapacity, Endianness LocalEndianness>
P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness>::P2PPacketStream(P2PByteStreamInterface<LocalEndianness> *byte_stream, TimerInterface *timer, GUIDFactoryInterface &guid_factory)
    : input_(byte_stream, timer), output_(byte_stream, timer), 
      handshake_id_(guid_factory.CreateGUID<kSequenceNumberNumBytes, kP2PLowestToken>()), handshake_done_(false) {
  for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
    last_init_sequence_number_[i] = -1ULL;
  }
  
  ResetInput();

  input_.packet_filter(P2PPacketFilter(&ShouldCommitInputPacket, this));
  output_.packet_filter(P2PPacketFilter(&ShouldConsumeOutputPacket, this));

  // Schedule handshake packet. The handshake reply is a regular ACK with is_init.
  P2PPriority init_priority = P2PPriority::kHigh;
  StatusOr<P2PMutablePacketView> init_packet_view = output_.NewPacket(init_priority);
  ASSERT(init_packet_view.ok());
  init_packet_view->packet()->header()->is_init = 1;
  output_.Commit(init_priority, /*guaranteed_delivery=*/true, /*seq_number=*/handshake_id_);
}

template<int kInputCapacity, int kOutputCapacity, Endianness LocalEndianness>
void P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness>::ResetInput() {
  input_.Reset();
  for (int i = 0; i < P2PPriority::kNumLevels; ++i) {
    last_rx_sequence_number_[i] = -1ULL;
  }
}

template<int kInputCapacity, int kOutputCapacity, Endianness LocalEndianness>
void P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness>::ResetOutputSession(const P2PPacket &handshake_request) {
  // Purge ACKs in output buffer (except for those of the ongoing handshake).
  for (int p = 0; p < P2PPriority::kNumLevels; ++p) {
    // Cache packets to copy because Size() could change within the loop.
    const int num_packets_to_maybe_copy = output_.packet_buffer_.Size(p);
    for (int i = 0; i < num_packets_to_maybe_copy; ++i) {
      const P2PPacket packet_copy = *output_.packet_buffer_.OldestValue(p);
      output_.packet_buffer_.Consume(p);
      if (!packet_copy.header()->is_ack ||
          packet_copy.sequence_number() == handshake_request.sequence_number()) {
        output_.packet_buffer_.NewValue(p) = packet_copy;
        output_.packet_buffer_.Commit(p);
      }
    }
  }

  // Reset continuation packets to the original packets.
  for (int p = 0; p < P2PPriority::kNumLevels; ++p) {
    P2PPacket *packet = output_.packet_buffer_.OldestValue(p);
    if (packet != NULL && packet->header()->is_continuation) {
      packet->header()->is_continuation = 0;
      ASSERT(output_.total_packet_bytes_[p] != -1);
      packet->length() = LocalToNetwork<LocalEndianness>(output_.total_packet_bytes_[p] - sizeof(P2PHeader) - sizeof(P2PFooter));
    }
  }
}

template<int kInputCapacity, int kOutputCapacity, Endianness LocalEndianness>
bool P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness>::IsACKCommittedForPacket(const P2PPacket &packet) {
  // ACKs always have a priority one level higher to avoid deadlocks.
  const P2PPriority ack_priority = packet.header()->priority - 1;
  bool ack_found = false;
  for (int i = 0; i < output_.packet_buffer_.Size(ack_priority); ++i) {
    const P2PPacket *maybe_ack_packet = output_.packet_buffer_.OldestValue(ack_priority, i);
    ASSERT(maybe_ack_packet != NULL);
    if (maybe_ack_packet->sequence_number() == packet.sequence_number()) {
      ack_found = true;
      break;
    }
  }      
  return ack_found;
}

template<int kInputCapacity, int kOutputCapacity, Endianness LocalEndianness>
bool P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness>::ScheduleACKWithThrottling(const P2PPacket &packet) {
  if (IsACKCommittedForPacket(packet)) {
    return true;
  }
  // ACKs always have a priority one level higher to avoid deadlocks.
  const P2PPriority ack_priority = packet.header()->priority - 1;
  StatusOr<P2PMutablePacketView> ack_packet_view = output_.NewPacket(ack_priority);
  if (!ack_packet_view.ok()) {
    // Only commit the packet if there is space for the ACK in the output stream.
    // Let the other end keep retransmitting until there's space for the ACK.
    return false;
  }
  P2PPacket *ack = ack_packet_view->packet();
  ack->header()->is_ack = 1;
  ack->header()->is_init = packet.header()->is_init;
  output_.Commit(ack_priority, /*guaranteed_delivery=*/false, /*seq_number=*/packet.sequence_number());
  return true;
}
#include <sstream>
template<int kInputCapacity, int kOutputCapacity, Endianness LocalEndianness>
bool P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness>::ShouldCommitInputPacket(const P2PPacket &last_rx_packet, void *self_ptr) {
  P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness> &self = *reinterpret_cast<P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness> *>(self_ptr);
  if (!self.handshake_done_) {
    if (!last_rx_packet.header()->is_init) { 
      // Reject all packets until handshake. This ensures that any old ACKs or continuation in
      // the other end's serial output byte buffer won't be processed, as they could have a
      // valid sequence number.
      return false;
    }
    if (last_rx_packet.header()->is_ack &&
    last_rx_packet.sequence_number() == self.handshake_id_) {
      // The other end replied to a handshake, and it is the one we last started.
      self.handshake_done_ = true;
    }
  }

  if (last_rx_packet.header()->is_init && !last_rx_packet.header()->is_ack) {
    // Handshake request: any state tied to the other end's state before reset is invalid:
    // a) Packets from the other end, received or in progress.
    // b) ACKs and continuations from this end, unless it's an ACK for the handshake in progress.
    // We can do (a) right here because we just received a full packet and state variables may be
    // dicarded, but we must wait until the end of the sender's packet in progress to do (b).
    self.ResetInput();

    // Notify listeners that the other end was restarted, only once.
    const P2PPriority priority = last_rx_packet.header()->priority;
    // The sequence number can never be equal to last_init_sequence_number at startup (-1) 
    // because its bytes are limited to the lowest possible token value (start, special) 
    // of the protocol.
    if (last_rx_packet.sequence_number() != self.last_init_sequence_number_[priority]) {      
      self.other_end_started_callback_();
    }
    self.last_init_sequence_number_[priority] = last_rx_packet.sequence_number();

    // The session was reset, so there should always be space in the output queue at the 
    // ACK's priority, if the handshake is at the highest priority.
    const bool ack_ok = self.ScheduleACKWithThrottling(last_rx_packet);
    ASSERT(ack_ok);

    return false;
  }

  if (last_rx_packet.header()->is_ack) {
    // We got an ACK: discard the retrainsmitting packet that originated it.

    // ACKs always have a priority one level higher to avoid deadlocks. Turn priority down one
    // notch to get that of the retransmitting packet.
    P2PPriority data_packet_priority = last_rx_packet.header()->priority + 1;

    const P2PPacket *retransmitting_packet = self.output_.packet_buffer_.OldestValue(data_packet_priority);      
    // Check the retransmitting packet exists, as it could have been consumed already by a
    // previous ACK.      
    if (retransmitting_packet != NULL && retransmitting_packet->header()->requires_ack &&
        last_rx_packet.sequence_number() == retransmitting_packet->sequence_number()) {
      self.output_.packet_buffer_.Consume(data_packet_priority);
    }

    // Do not expose an ACK in the API.
    return false;
  }

  // It's a data packet: reply with ACK if there is no ACK in the output buffer already,
  // to avoid flooding the buffer and blocking the sender for this priority and lower.
  if (last_rx_packet.header()->requires_ack) {

    if (!self.ScheduleACKWithThrottling(last_rx_packet)) {
      // No space for the ACK packet: let the other end retransmit until we can guarantee the
      // ACK is sent.
      return false;
    }

    const P2PPriority priority = last_rx_packet.header()->priority;
    if (self.last_rx_sequence_number_[priority] != -1ULL &&
        last_rx_packet.sequence_number() <= self.last_rx_sequence_number_[priority]) {
      // This packet had been received already: filter it.
      return false;
    }
    self.last_rx_sequence_number_[priority] = last_rx_packet.sequence_number();
  }
  
  // Expose the packet in the API.
  return true;
}

template<int kInputCapacity, int kOutputCapacity, Endianness LocalEndianness>
bool P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness>::ShouldConsumeOutputPacket(const P2PPacket &last_tx_packet, void *self_ptr) {
  P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness> &self = *reinterpret_cast<P2PPacketStream<kInputCapacity, kOutputCapacity, LocalEndianness> *>(self_ptr);
  if (last_tx_packet.header()->is_init && last_tx_packet.header()->is_ack) {
    // The handshake ACK was just sent. We may now reset the output session without
    // having to update the packet-scope state variables.
    self.ResetOutputSession(last_tx_packet);
  }
  // Packets requiring an ACK are left in the queue for retransmission.
  return !last_tx_packet.header()->requires_ack;
}

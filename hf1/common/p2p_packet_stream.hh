#include <algorithm>

template<int kCapacity, Endianness LocalEndianness> void P2PPacketInputStream<kCapacity, LocalEndianness>::Run() {
  switch (state_) {
    case kWaitingForPacket:
      {
        if (byte_stream_.Read(&incoming_header_.start_token, 1) < 1) {
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
            //Serial.println("invalid priority.");
            state_ = kWaitingForPacket;
            break;
          }
          P2PPacket &packet = packet_buffer_.NewValue(incoming_header_.priority);
          if (!incoming_header_.is_continuation) {
            //Serial.println("New packet.");
            for (unsigned int i = 0; i < sizeof(P2PHeader); ++i) {
              reinterpret_cast<uint8_t *>(&packet)[i] = reinterpret_cast<uint8_t *>(&incoming_header_)[i];
            }
            // Fix endianness of header fields, so they can be used locally in next states.
            packet.length() = NetworkToLocal<LocalEndianness>(packet.length());
            packet.sequence_number() = NetworkToLocal<LocalEndianness>(packet.sequence_number());
            current_field_read_bytes_ = 0;
          } else {
            //Serial.println("Packet continuation.");
            // The length field for a packet continuation is the remaining length.
            int remaining_length = NetworkToLocal<LocalEndianness>(incoming_header_.length);
            if (NetworkToLocal<LocalEndianness>(incoming_header_.sequence_number) != packet.header()->sequence_number || 
                remaining_length > packet.length()) {
              // This continuation does not belong to the packet we have in store. There must
              // have been a link interruption: reset the state machine.
              //Serial.println("bad continuation");
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
        int read_bytes = byte_stream_.Read(current_byte, 1);
        if (read_bytes < 1) {
          break;
        }
        current_field_read_bytes_ += read_bytes;
        if (*current_byte == kP2PStartToken) {
          // Must be a new packet after a link interruption because priority takeover is
          // not legal mid-header.
          state_ = kReadingHeader;
          incoming_header_.start_token = kP2PStartToken;
          current_field_read_bytes_ = 1;
          //Serial.println("new packet when reading header");
          break;
        }
        if (*current_byte == kP2PSpecialToken) {
          // Malformed packet.
          //Serial.println("malformed packet when reading header");
          state_ = kWaitingForPacket;
        }
        break;
      }

    case kReadingContent:
      {
        P2PPacket &packet = packet_buffer_.NewValue(incoming_header_.priority);
        if (current_field_read_bytes_ >= packet.length()) {
          state_ = kReadingFooter;
          current_field_read_bytes_ = 0;
          break;
        }
        uint8_t *next_content_byte = &packet.content()[current_field_read_bytes_];
        int read_bytes = byte_stream_.Read(next_content_byte, 1);
        if (read_bytes < 1) {
          break;
        }
        current_field_read_bytes_ += read_bytes;
        if (*next_content_byte == kP2PStartToken) {
          if (current_field_read_bytes_ < packet.length()) {
          //Serial.println("new start token when reading content");
            // It could be a start token, if the next byte is not a special token.
            state_ = kDisambiguatingStartTokenInContent;
          } else {
            // If there can't be a special token next because this is the last content byte,
            // it could either be a malformed packet or a new packet start. Assume the other
            // end will form correct packets. The start token may then be due to a new packet
            // after a link interruption, or a packet with higher priority.
          //Serial.println("new packet when reading content");
            state_ = kReadingHeader;
            incoming_header_.start_token = kP2PStartToken;
            current_field_read_bytes_ = 1;
          }
        }
        break;
      }

    case kDisambiguatingStartTokenInContent:
      {
        P2PPacket &packet = packet_buffer_.NewValue(incoming_header_.priority);
        // Read next byte and check if it's a special token.
        // No need to check if we reached the content length here, as a content byte matching the start token should always be followed by a special token.
        uint8_t *next_content_byte = &packet.content()[current_field_read_bytes_];
        int read_bytes = byte_stream_.Read(next_content_byte, 1);
        if (read_bytes < 1) {
          break;
        }
        current_field_read_bytes_ += read_bytes;
        if (*next_content_byte == kP2PSpecialToken) {
          // Not a start token, but a content byte.
          //Serial.println("start token was a content byte");
          state_ = kReadingContent;
        } else {
          if (*next_content_byte == kP2PStartToken) {
            // Either a malformed packet, or a new packet after link reestablished, or a
            // higher priority packet.
            // Assume well designed transmitter and try the latter.
          //Serial.println("new packet after start token when reading content");
            state_ = kReadingHeader;
            incoming_header_.start_token = kP2PStartToken;
            current_field_read_bytes_ = 1;
          } else {
          //Serial.println("start token was a new packet");
            // Must be a start token. Restart the state to re-synchronize with minimal latency.
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
        P2PPacket &packet = packet_buffer_.NewValue(incoming_header_.priority);
        if (current_field_read_bytes_ >= sizeof(P2PFooter)) {
          state_ = kWaitingForPacket;
          break;
        }
        uint8_t *current_byte = packet.content() + packet.length() + current_field_read_bytes_;
        int read_bytes = byte_stream_.Read(current_byte, 1);
        if (read_bytes < 1) {
          break;
        }
        current_field_read_bytes_ += read_bytes;

        if (*current_byte == kP2PStartToken) {
          // New packet after interrupts, as no priority takeover is allowed mid-footer.
          //Serial.println("new packet when reading footer");
          state_ = kReadingHeader;
          incoming_header_.start_token = kP2PStartToken;
          current_field_read_bytes_ = 1;
          break;
        }
        if (*current_byte == kP2PSpecialToken) {
          // Malformed packet.
          //Serial.println("malformed packet when reading footer");
          state_ = kWaitingForPacket;
        }

        if (current_field_read_bytes_ >= sizeof(P2PFooter)) {
          // Fix endianness of footer fields.
          packet.checksum() = NetworkToLocal<LocalEndianness>(packet.checksum());
          if (packet.PrepareToRead()) {
            packet_buffer_.Commit(incoming_header_.priority);
          }
          state_ = kWaitingForPacket;
        }
      }
      break;
  }
}

template<int kCapacity, Endianness LocalEndianness> uint64_t P2PPacketOutputStream<kCapacity, LocalEndianness>::Run(uint64_t timestamp_ns) {
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
        }
        pending_packet_bytes_ = sizeof(P2PHeader);

        // for (int i = 0; i < total_packet_bytes_[priority]; ++i) {
          //Serial.printf("%x ", reinterpret_cast<uint8_t *>(current_packet_->header())[i]);
        // }
        //Serial.printf("\n");

        state_ = kSendingHeaderBurst;
        total_burst_bytes_ = std::min(pending_packet_bytes_, byte_stream_.GetBurstMaxLength());
        pending_burst_bytes_ = total_burst_bytes_;
        break;
      }

    case kSendingHeaderBurst: {
      int written_bytes = byte_stream_.Write(
        &reinterpret_cast<const uint8_t *>(current_packet_->header())[sizeof(P2PHeader) - pending_packet_bytes_],
        pending_burst_bytes_);
      pending_packet_bytes_ -= written_bytes;
      pending_burst_bytes_ -= written_bytes;

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
        int written_bytes = byte_stream_.Write(
          &reinterpret_cast<const uint8_t *>(current_packet_->header())[total_packet_bytes_[priority] - pending_packet_bytes_],
          std::min(byte_stream_.GetAtomicSendMaxLength(), pending_burst_bytes_));
        pending_packet_bytes_ -= written_bytes;
        pending_burst_bytes_ -= written_bytes;

        if (pending_packet_bytes_ <= 0) { 
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
        if (timestamp_ns < after_burst_wait_end_timestamp_ns_) {
          // Ingestion time not expired: keep waiting.
          time_until_next_event = after_burst_wait_end_timestamp_ns_ - timestamp_ns;
          break;
        }

        // Burst should have been ingested by the other end.
        if (pending_packet_bytes_ <= 0) {
          // No more bursts: next packet.
          packet_buffer_.Consume(current_packet_->header()->priority);
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

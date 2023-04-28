template<int kCapacity, Endianness LocalEndianness> void P2PPacketInputStream<kCapacity, LocalEndianness>::Run() {
  switch (state_) {
    case kWaitingForPacket:
      {
        uint8_t *start_token = &packet_buffer_.NewValue().header()->start_token;
        if (byte_stream_.Read(start_token, 1) < 1) {
          break;
        }
        if (*start_token == kP2PStartToken) {
          state_ = kReadingHeader;
          current_field_read_bytes_ = 1;
        }
        break;
      }

    case kReadingHeader:
      {
        if (current_field_read_bytes_ >= sizeof(P2PHeader)) {
          state_ = kReadingContent;
          current_field_read_bytes_ = 0;
          break;
        }
        uint8_t *current_byte = &reinterpret_cast<uint8_t *>(packet_buffer_.NewValue().header())[current_field_read_bytes_];
        int read_bytes = byte_stream_.Read(current_byte, 1);
        if (read_bytes < 1) {
          break;
        }
        current_field_read_bytes_ += read_bytes;
        if (*current_byte == kP2PStartToken) {
          state_ = kReadingHeader;
          current_field_read_bytes_ = 1;
          break;
        }
        if (*current_byte == kP2PSpecialToken) {
          state_ = kWaitingForPacket;
        }
        break;
      }

    case kReadingContent:
      {
        if (current_field_read_bytes_ >= packet_buffer_.NewValue().length()) {
          state_ = kReadingFooter;
          current_field_read_bytes_ = 0;
          break;
        }
        uint8_t *next_content_byte = &packet_buffer_.NewValue().content()[current_field_read_bytes_];
        int read_bytes = byte_stream_.Read(next_content_byte, 1);
        if (read_bytes < 1) {
          break;
        }
        current_field_read_bytes_ += read_bytes;
        if (*next_content_byte == kP2PStartToken) {
          if (current_field_read_bytes_ < packet_buffer_.NewValue().length()) {
            // It could be a start token, if the next byte is not a special token.
            state_ = kDisambiguatingStartTokenInContent;
          } else {
            // If there can't be a special token next because this is the last content byte,
            // it could either be a malformed packet or a new packet start after the link
            // was reestablished. Assume the former will not happen and try the latter.
            state_ = kReadingHeader;
            current_field_read_bytes_ = 1;
          }
        }
        break;
      }

    case kDisambiguatingStartTokenInContent:
      {
        // Read next byte and check if it's a special token.
        // No need to check if we reached the content length here, as a content byte matching the start token should always be followed by a special token.
        uint8_t *next_content_byte = &packet_buffer_.NewValue().content()[current_field_read_bytes_];
        int read_bytes = byte_stream_.Read(next_content_byte, 1);
        if (read_bytes < 1) {
          break;
        }
        current_field_read_bytes_ += read_bytes;
        if (*next_content_byte == kP2PSpecialToken) {
          // Not a start token, but a content byte.
          state_ = kReadingContent;
        } else {
          if (*next_content_byte == kP2PStartToken) {
            // Either a malformed packet or a new packet start after link reestablished.
            // Assume well designed transmitter and try the latter.
            state_ = kReadingHeader;
            current_field_read_bytes_ = 1;
          } else {
            // Must be a start token. Restart the state to re-synchronize with minimal latency.
            state_ = kReadingHeader;
            packet_buffer_.NewValue().header()->start_token = kP2PStartToken;
            reinterpret_cast<uint8_t *>(packet_buffer_.NewValue().header())[1] = *next_content_byte;
            current_field_read_bytes_ = 2;
          }
        }
      }
      break;

    case kReadingFooter:
      if (current_field_read_bytes_ >= sizeof(P2PFooter)) {
        state_ = kWaitingForPacket;
        break;
      }
      uint8_t *current_byte = packet_buffer_.NewValue().content() + packet_buffer_.NewValue().length() + current_field_read_bytes_;
      int read_bytes = byte_stream_.Read(current_byte, 1);
      if (read_bytes < 1) {
        break;
      }
      current_field_read_bytes_ += read_bytes;

      if (*current_byte == kP2PStartToken) {
        state_ = kReadingHeader;
        current_field_read_bytes_ = 1;
        break;
      }
      if (*current_byte == kP2PSpecialToken) {
        state_ = kWaitingForPacket;
      }

      if (current_field_read_bytes_ >= sizeof(P2PFooter)) {
        // Fix endianness.
        packet_buffer_.NewValue().checksum() = NetworkToLocal<LocalEndianness>(packet_buffer_.NewValue().checksum());
        packet_buffer_.NewValue().length() = NetworkToLocal<LocalEndianness>(packet_buffer_.NewValue().length());
        if (packet_buffer_.NewValue().PrepareToRead()) {
          packet_buffer_.Commit();
        }
        state_ = kWaitingForPacket;
      }
      break;
  }
}

template<int kCapacity, Endianness LocalEndianness> void P2PPacketOutputStream<kCapacity, LocalEndianness>::Run(uint64_t timestamp_ns) {
  switch (state_) {
    case kGettingNextPacket:
      {
        const P2PPacket *packet = packet_buffer_.OldestValue();
        if (packet == NULL) {
          // No more packets to send: keep waiting for one.
          break;
        }

        // Start sending the new packet.
        state_ = kSendingBurst;
        total_packet_length_ = sizeof(P2PHeader) + NetworkToLocal<LocalEndianness>(packet->length()) + sizeof(P2PFooter);

        // Send first burst.
        // First write in the state transition so that burst_end_timestamp_ns_ is calculated
        // as close as possible to it.
        int burst_length = std::min(total_packet_length_, byte_stream_.GetBurstMaxLength());
        burst_end_timestamp_ns_ = timestamp_ns + burst_length * byte_stream_.GetBurstIngestionNanosecondsPerByte();
        int written_bytes = byte_stream_.Write(packet->header(), burst_length);
        pending_packet_bytes_ = total_packet_length_ - written_bytes;
        pending_burst_bytes_ = burst_length - written_bytes;
        
        break;
      }
    
    case kSendingBurst:
      {
        if (pending_burst_bytes_ <= 0) {
          // Burst fully sent: wait for other end to ingest.
          state_ = kWaitingForBurstIngestion;
          break;
        }
        int written_bytes = byte_stream_.Write(&reinterpret_cast<const uint8_t *>(packet_buffer_.OldestValue()->header())[total_packet_length_ - pending_packet_bytes_], pending_burst_bytes_);
        pending_packet_bytes_ -= written_bytes;
        pending_burst_bytes_ -= written_bytes;

        break;
      }

    case kWaitingForBurstIngestion:
      if (timestamp_ns < burst_end_timestamp_ns_) {
        // Ingestion time not expired: keep waiting.
        break;
      }

      // Burst should have been ingested by the other end.
      if (pending_packet_bytes_ <= 0) {
        // No more bursts: next packet.
        packet_buffer_.Consume();
        state_ = kGettingNextPacket;
        break;
      }

      // Send next burst.
      // First write in the state transition so that burst_end_timestamp_ns_ is calculated
      // as close as possible to it.
      state_ = kSendingBurst;
      pending_burst_bytes_ = std::min(pending_packet_bytes_, byte_stream_.GetBurstMaxLength());
      burst_end_timestamp_ns_ = timestamp_ns + pending_burst_bytes_ * byte_stream_.GetBurstIngestionNanosecondsPerByte();
      int written_bytes = byte_stream_.Write(&reinterpret_cast<const uint8_t *>(packet_buffer_.OldestValue()->header())[total_packet_length_ - pending_packet_bytes_], pending_burst_bytes_);
      pending_packet_bytes_ -= written_bytes;
      pending_burst_bytes_ -= written_bytes;

      break;
  }
}

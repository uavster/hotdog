template<int kCapacity, Endianness LocalEndianness> void P2PPacketInputStream<kCapacity, LocalEndianness>::Run() {
    switch(state_) {
      case kWaitingForPacket:
        if (byte_stream_.ReadByteOrDefault(static_cast<uint8_t>(~kP2PStartToken)) == kP2PStartToken) {
          packet_buffer_.NewValue().header()->start_token = kP2PStartToken;
          state_ = kVerifyingStartToken;
          current_field_read_bytes_ = 1;
        Serial.println("0");
        }
        break;

      case kVerifyingStartToken:
        // The token is only an actual start token if the following byte is not a special token.
        current_field_read_bytes_ += byte_stream_.Read(&packet_buffer_.NewValue().length(), 1); // Read only 1 regardless of field size.
        if (current_field_read_bytes_ >= 1) {
          if (reinterpret_cast<uint8_t *>(&packet_buffer_.NewValue().length())[0] == kP2PSpecialToken) {
            // Not a new packet, but part of the content of an ongoing packet: skip.
        Serial.println("1");
            state_ = kWaitingForPacket;
          } else {
            // New packet: keep reading.
        Serial.println("2");
            state_ = kReadingHeader;
          }
        }
        break;

      case kReadingHeader:
        current_field_read_bytes_ += byte_stream_.Read(packet_buffer_.NewValue().header(), sizeof(P2PHeader) - current_field_read_bytes_);
        Serial.printf("3: %d\n", (int)current_field_read_bytes_);
        if (current_field_read_bytes_ == sizeof(P2PHeader)) {          
        Serial.println("4");
          state_ = kReadingContent;
          current_field_read_bytes_ = 0;
        }
        break;
      
      case kReadingContent:
        Serial.printf("5: %d\n", packet_buffer_.NewValue().length());
        current_field_read_bytes_ += byte_stream_.Read(packet_buffer_.NewValue().content(), packet_buffer_.NewValue().length() - current_field_read_bytes_);
        if (current_field_read_bytes_ == packet_buffer_.NewValue().length()) {
        Serial.printf("6: %d\n", current_field_read_bytes_);
          state_ = kReadingFooter;
          current_field_read_bytes_ = 0;
        }
        break;
      
      case kReadingFooter:
        Serial.printf("7: %d\n", (int)(sizeof(P2PFooter) - current_field_read_bytes_));
        current_field_read_bytes_ += byte_stream_.Read(packet_buffer_.NewValue().content() + packet_buffer_.NewValue().length(), sizeof(P2PFooter) - current_field_read_bytes_);
        if (current_field_read_bytes_ >= sizeof(P2PFooter)) {
        Serial.println("8");
          packet_buffer_.NewValue().checksum() = NetworkToLocal<LocalEndianness>(packet_buffer_.NewValue().checksum());
          if (packet_buffer_.NewValue().PrepareToRead()) {
            packet_buffer_.Commit();
          }
          state_ = kWaitingForPacket;
        }
        break;

    }
}

template<int kCapacity, Endianness LocalEndianness> void P2PPacketOutputStream<kCapacity, LocalEndianness>::Run() {
  switch(state_) {
    case kGettingNextPacket: {
      if (packet_buffer_.OldestValue() != NULL) {
        state_ = kSendingPacket;
        const P2PPacket *packet = packet_buffer_.OldestValue();
        pending_packet_bytes = packet->TotalLength() - byte_stream_.Write(packet->header(), packet->TotalLength());
      }
      break;
    }
    case kSendingPacket: {
      pending_packet_bytes -= byte_stream_.Write(packet_buffer_.OldestValue()->header(), pending_packet_bytes);
      if (pending_packet_bytes <= 0) {
        packet_buffer_.Consume();
        state_ = kGettingNextPacket;
      }
      break;
    }
  }
}
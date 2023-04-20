#include "p2p_packet_stream.h"
#include <stddef.h>

P2PChecksumType P2PPacket::CalculateChecksum() const {
  P2PChecksumType sum = 0;
  for (unsigned int i = 0; i < sizeof(data_.header); ++i) {
    sum += reinterpret_cast<const uint8_t *>(&data_.header)[i];
  }
  for (int i = 0; i < length(); ++i) {
    sum += content()[i];
  }
  sum -= kP2PStartToken;
  return sum % kP2PChecksumModulo;
}

bool P2PPacket::PrepareToRead() {
  Serial.printf("%d == %d\n", CalculateChecksum(), checksum());
  if (CalculateChecksum() != checksum()) {
    return false;
  }
  int read_index = 0;
  int write_index = 0;
  if (length() == 0) {
    return true;
  }
  if (length() == 1) {
    if (content()[read_index] == kP2PStartToken || content()[read_index] == kP2PSpecialToken) {
      return false;
    }
  }
  while(read_index < length()) {
    if (content()[read_index] == kP2PStartToken || content()[read_index] == kP2PSpecialToken) {
      if (content()[read_index + 1] != kP2PSpecialToken) {
        return false;
      } else {
        content()[write_index] = content()[read_index];
        read_index += 2;
        ++write_index;
      }
    } else {
      content()[write_index] = content()[read_index];
      ++read_index;
      ++write_index;
    }
  }
  length() = write_index;
  return true;
}

bool P2PPacket::PrepareToSend() {
  int read_index = 0;
  int write_index = 0;
  header()->start_token = kP2PStartToken;
  while(read_index < length()) {
    content()[write_index] = content()[read_index];
    ++read_index;
    ++write_index;
    if (write_index > kP2PMaxContentLength) { return false; }
    if (content()[read_index] == kP2PStartToken || content()[read_index] == kP2PSpecialToken) {
      content()[write_index] = kP2PSpecialToken;
      ++write_index;
      if (write_index > kP2PMaxContentLength) { return false; }
    }
  }
  length() = write_index;
  checksum() = CalculateChecksum();
  return true;
}
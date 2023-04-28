// Protocol to transfer packets reliably.
//
// This protocol is implemented on top of regular P2P packets. It guarantees packet delivery
// by periodically sending each packet until an ACK with its sequence number is received. The
// other end discards any duplicates after the first packet is received and replies with one
// ACK per duplicate.

#ifndef P2P_RELIABLE_PROTOCOL__
#define P2P_RELIABLE_PROTOCOL__

#include <stdint.h>

#pragma pack(push, 1)

typedef struct {
  uint16_t is_ack: 1;  // 0 = Data packet (data follows), 1 = ACK (no data follows).
  uint16_t seq_number: 15;  // Increments monotonically with each data packet. Matches each packet with its ACK.
} P2PReliablePacketHeader;

#pragma pack(pop)

#endif  // P2P_RELIABLE_PROTOCOL__
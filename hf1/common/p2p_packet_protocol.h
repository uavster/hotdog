// Protocol to transfer variable-length packets through a possibly unstable point-to-point link.
//
// Both ends may be physically connected to the medium at different times, and the medium may
// experience interruptions. The protocol increases the probability that received packets are
// actual packets, and their content is as sent. Optionally, it can guarantee that sent packets are
// received at the other end once the link is reestablished.
//
// Packets can be prioritized according to the urgency of their content, in which case, the protocol
// guarantees that higher priority packets will have minimal delay by preempting any lower level
// packets in progress. 
//
// Synchronization
// ---------------
// The protocol implements byte-level synchronization via signaling to minimize fake packets
// (sequences in a packet's content looking like complete packets themselves), and minimizes
// latency of re-synchronization after link glitches. It relies on the lower level of the
// communication stack to provide bit-level synchronization (e.g. UART).
//
// Packets begin with a start token. A special token is appended to content bytes matching the start
// and special tokens. Non-content bytes cannot match either token. This ensures that the protocol
// handling logic can identify the next packet with minimal latency after the link is reestablished.
// Because of that, the ranges of the bytes in non-content fields are limited by the token values.
// Therefore, choosing the tokens at either end of they byte range (0 or 0xff) keeps the range of
// valid values continuous in byte fields, which simplifies handling.
//
// For instance, if ^ and \ are the start and special tokens, content bytes matching them would be
// respectively encoded as ^\ and \\ before transmission. Starting a packet with ^\ would be illegal;
// this avoids the ambiguity between a content symbol equal to the start token and an actual packet with
// a length equal to the special token.
//
// Data integrity
// --------------
// The protocol maximizes packet integrity with length and checksum fields.
//
// As a non-content field, the checksum cannot match either token, and so we must compute it as:
// checksum  = modulo(sum(content_bytes) + sum(header_bytes) - start_token, M)
//   where M < min(start_token, special_token)
// And, so, we pay a price in error detection power; especially, if we have to optimize the modulo
// operation by choosing an M that is a power of 2. In that situation, however, we have a wider range
// to choose from for the token values, and may use the opportunity to pick a start token value that
// is infrequent among erroneous bytes.
//
// Priority
// --------
// A packet with a higher priority (lower value in priority field) will preempt a lower priority
// packet being sent. Once the high priority packet is fully sent, the transmitter will resume
// sending the preempted packet. At the receiver end, the high priority packet will be delivered to
// the application before all lower priority packets.
//
// Guaranteed delivery
// -------------------
// If a packet is marked as reliable (requires_ack=1), the sender will peridically re-send it until
// an acknowledge (is_ack == 1) from the other end is received. While waiting for an ACK for a
// packet with priority P, packets with priority P-1 will still go throught the link, but packets
// whose priority is >= P, will be blocked.
//
// ACK packets should have the priority of the original packet minus 1. This is to prevent a
// deadlock when the two ends send reliable packets of the same priority at the same time.
// In that case, each end will put one packet in its send queue and will wait for an ACK to remove
// it from the queue, but that will never happen because the other end will put the ACK in the
// send queue too, which will be blocked by the other packet waiting to be acknowledged. 

#ifndef P2P_PROTOCOL__
#define P2P_PROTOCOL__

#include <stdint.h>

#define kP2PStartToken 0xaa
#define kP2PSpecialToken 0xff
#define kP2PChecksumModulo 0x80

#if (kP2PStartToken == kP2PSpecialToken)
#error "The start and special tokens must be different."
#endif

#if (kP2PStartToken < kP2PChecksumModulo)
#error "The checksum might match the start token, which is forbidden. Please make kP2PStartToken greater than kP2PStartToken."
#endif

#if (kP2PSpecialToken < kP2PChecksumModulo)
#error "The checksum might match the start token, which is forbidden. Please make kP2PSpecialToken greater than kP2PStartToken."
#endif

#if (kP2PStartToken < kP2PSpecialToken)
#define kP2PLowestToken kP2PStartToken
#else
#define kP2PLowestToken kP2PSpecialToken
#endif

#define kP2PMaxContentLength static_cast<uint8_t>(kP2PLowestToken - 1)

typedef uint8_t P2PChecksumType;

// This must be chosen so that the highest sequence number period is always below the packet
// timeout or link watchdog. The highest period is:
// maximum_packet_frequency * kP2PLowestToken^kSequenceNumberNumBytes
#define kSequenceNumberNumBytes 3

#pragma pack(push, 1)

typedef struct {
  // Must be kP2PStartToken.
  uint8_t start_token;

  // Allocation of bits is implementation-dependent. Care must be taken to ensure that the following bit fields
  // are packed from least to most significant in all platforms.

  // Priority of the packet (0 is highest). In both link ends, a packet with higher priority will
  // preempt packets of lower priority.
  uint8_t priority: 2;

  // If 1, the packet is the continuation of a previous packet that was preempted by a higher
  // priority packet. In that case, the length field is the remaining length, and the offset of
  // the content bytes is legth_of_original_packet - length_of_continuation_packet.
  uint8_t is_continuation: 1;

  // 0 = no acknowledge needed, 1 = the packet must be acknowledged.
  uint8_t requires_ack: 1;

  // 0 = Data packet (data follows), 1 = ACK packet (no data follows).
  uint8_t is_ack: 1;

  // 0 = regular packet, 1 = first packet after program restarts (all packets from the other 
  // end received before the init's ACK should be discarded).
  uint8_t is_init: 1;

  // The reserved field must not match the corresponding bits in either token.
  uint8_t reserved: 2;

  // The sequence number increments monotonically with each data packet. Each priority
  // level has its own sequence number. It is used to pair every continuation and ACK with
  // the original packet.
  // No byte in this field can match a token, so the total representable values is
  // kP2PLowestToken^kSequenceNumberNumBytes
  // It is little-endian.
  uint8_t sequence_number[kSequenceNumberNumBytes];

  // Number of content bytes, including special characters. Cannot match any token.
  uint8_t length;
} P2PHeader;

typedef struct {
  // checksum  = modulo(sum(content_bytes) + sum(header_bytes) - kP2PStartToken, kP2PChecksumModulo)
  // It can't match any token.
  P2PChecksumType checksum;
} P2PFooter;

#pragma pack(pop)

#endif  // P2P_PROTOCOL__
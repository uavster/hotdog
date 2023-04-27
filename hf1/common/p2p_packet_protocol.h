// Protocol to transfer variable-length packets through a possibly unstable point-to-point link.
//
// Both ends may be physically connected to the medium at different times, and the medium may experience
// interruptions. The protocol increases the probability that received packets are actual packets and
// their content is as sent.
//
// The first layer of the protocol implements byte-level synchronization via signaling to minimize
// fake packets (sequences in a packet's content looking like complete packets themselves), and
// minimizes latency of re-synchronization after link glitches. It relies on the lower level of the
// communication stack to provide bit-level synchronization (e.g. UART).
//
// The second layer maximizes packet integrity with length and checksum fields.
//
// Packets begin with a start token. A special token is appended to content bytes matching the start
// and special tokens. Non-content bytes cannot match either token. This ensures that the protocol
// handling logic can identify the next packet with minimal latency after the link is reestablished.
//
// For instance, if ^ and \ are the start and special tokens, content bytes matching them would be
// respectively encoded as ^\ and \\ before transmission. Starting a packet with ^\ would be illegal;
// this avoids the ambiguity between a content symbol equal to the start token and an actual packet with
// a length equal to the special token.
//
// Choosing the start and special tokens at either end of the length and checksum field bit depths
// (0 or -1) maximizes the representable content length, and keeps the space of valid values of these
// fields continuous, simplifying handling. Since the idle state of the channel is similar to
// transmitting a 0, choosing -1 as special token should be more robust to link interruptions.
//
// As a non-content field, the checksum cannot match either token, and so we must compute it as:
// checksum  = modulo(sum(content_bytes) + sum(header_bytes) - start_token, M)
//   where M < min(start_token, special_token)
// And, so, we pay a price in error detection power; especially, if we have to optimize the modulo
// operation by choosing an M that is a power of 2. In that situation, however, we have a wider range
// to choose from for the token values, and may use the opportunity to pick a start token value that
// is infrequent among erroneous bytes.

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
#error "The checksum may match the start token, which is forbidden. Please make kP2PStartToken greater than kP2PStartToken."
#endif

#if (kP2PSpecialToken < kP2PChecksumModulo)
#error "The checksum may match the start token, which is forbidden. Please make kP2PSpecialToken greater than kP2PStartToken."
#endif

#if (kP2PStartToken < kP2PSpecialToken)
#define kP2PMaxContentLength static_cast<uint8_t>(kP2PStartToken - 1)
#else
#define kP2PMaxContentLength static_cast<uint8_t>(kP2PSpecialToken - 1)
#endif

typedef uint8_t P2PChecksumType;

#pragma pack(push, 1)

typedef struct {
  uint8_t start_token;
  // Number of content bytes, including special characters. Cannot match the special token.
  // If a larger type is needed, make the byte following start_token reserved,
  // and set it to something different from the start and special tokens.
  uint8_t length;
} P2PHeader;

typedef struct {
  // checksum  = modulo(sum(content_bytes) + sum(header_bytes) - kP2PStartToken, kP2PChecksumModulo)
  P2PChecksumType checksum;
} P2PFooter;

#pragma pack(pop)

#endif  // P2P_PROTOCOL__
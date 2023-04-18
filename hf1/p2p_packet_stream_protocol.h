// Protocol to transfers variable length packets through a possibly unstable point-to-point link.
//
// Both ends may be physically connected to the medium at different times, and the medium may experience
// interruptions. The protocol increases the probability that received packets are actual packets and
// their content is as sent.
//
// The first layer of the protocol implements synchronization via signaling to minimize fake packets:
// sequences in a packet's content looking like complete packets themselves. The second layer maximizes
// packet integrity with length and checksum fields.
//
// Packets begin with a start token. A special token is appended to content bytes matching the start
// and special tokens. The byte following the start token cannot be a special token. In our case, that
// byte is the content length. Choosing a special token at either end of the length range (0 or -1)
// keeps the space of valid lengths continuous, which simplifies handling. Since the idle state of the
// channel is similar to transmitting a 0, choosing -1 as special token should be more robust to
// interruptions.
//
// For instance, if ^ and \ are the start and special tokens, content bytes matching them would be
// respectively encoded as ^\ and \\ before transmission. Starting a packet with ^\ would be illegal;
// this avoids the ambiguity between a content symbol equal to the start token and an actual packet with
// a length equal to the special token.

#ifndef P2P_PROTOCOL__
#define P2P_PROTOCOL__

#include <stdint.h>

#define kP2PStartToken 0xaa
#define kP2PSpecialToken 0xff

// Maximum content length. 
// If a larger type is needed, make the byte following start_token reserved,
// and set it to something different from the special token.
#define kP2PMaxContentLength static_cast<uint8_t>(kP2PSpecialToken - 1)

typedef uint8_t P2PChecksumType;

#pragma pack(push, 1)

typedef struct {
  uint8_t start_token;
  // Number of content bytes, icluding special characters. Cannot match the special token.
  // If a larger type is needed, make the byte following start_token reserved,
  // and set it to something different from the special token.
  uint8_t length;
} P2PHeader;

typedef struct {
  P2PChecksumType checksum; // Modulo sum of all content bytes, including special tokens.
} P2PFooter;

#pragma pack(pop)

#endif  // P2P_PROTOCOL__
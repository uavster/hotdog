#ifndef P2P_PACKET_STREAM_LINUX_INCLUDED__
#define P2P_PACKET_STREAM_LINUX_INCLUDED__

#include "p2p_packet_stream.h"

#define kP2PInputCapacity 16
#define kP2POutputCapacity 16
#define kP2PLocalEndianness kLittleEndian

using P2PPacketStreamLinux = P2PPacketStream<kP2PInputCapacity, kP2POutputCapacity, kP2PLocalEndianness>;

#endif  // P2P_PACKET_STREAM_LINUX_INCLUDED__
#ifndef P2P_PACKET_STREAM_ARDUINO_
#define P2P_PACKET_STREAM_ARDUINO_

#include "p2p_packet_stream.h"

#define kP2PInputCapacity 4
#define kP2POutputCapacity 1
#define kP2PLocalEndianness kLittleEndian

using P2PPacketStreamArduino = P2PPacketStream<kP2PInputCapacity, kP2POutputCapacity, kP2PLocalEndianness>;

#endif  // P2P_PACKET_STREAM_ARDUINO_
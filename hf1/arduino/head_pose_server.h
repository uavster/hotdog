#ifndef HEAD_POSE_SERVER_
#define HEAD_POSE_SERVER_

#include "p2p_packet_stream_arduino.h"

class HeadPoseServer {
public:
  // Does not take ownsership of the pointee, which must outlive this object.
  HeadPoseServer(P2PPacketStreamArduino *p2p_server);

  void Run();

private:
  P2PPacketStreamArduino &p2p_stream_;
};

#endif  // HEAD_POSE_SERVER_
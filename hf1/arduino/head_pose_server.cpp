#include "head_pose_server.h"
#include "utils.h"
#include "p2p_application_protocol.h"
#include "servos.h"

HeadPoseServer::HeadPoseServer(P2PPacketStreamArduino *p2p_stream)
  : p2p_stream_(*ASSERT_NOT_NULL(p2p_stream)) {}

void HeadPoseServer::Run() {
  const auto maybe_oldest_packet_view = p2p_stream_.input().OldestPacket();
  if (!maybe_oldest_packet_view.ok() || 
      maybe_oldest_packet_view->length() < sizeof(P2PApplicationPacketHeader) || 
      reinterpret_cast<const P2PApplicationPacketHeader *>(maybe_oldest_packet_view->content())->command != kP2PCommandSetHeadPose) { 
      return;
  }

  ASSERT(maybe_oldest_packet_view->length() == sizeof(P2PApplicationPacketHeader) + sizeof(P2PSetHeadPoseRequestContent));

  // The edge detection latency on this computer is negligible compared to that on the Linux.
  const P2PSetHeadPoseRequestContent *params = reinterpret_cast<const P2PSetHeadPoseRequestContent *>(&maybe_oldest_packet_view->content()[sizeof(P2PApplicationPacketHeader)]);
  const float pitch_radians = NetworkToLocal<kP2PLocalEndianness>(params->pitch_radians);
  const float roll_radians = NetworkToLocal<kP2PLocalEndianness>(params->roll_radians);
  p2p_stream_.input().Consume(maybe_oldest_packet_view->priority());

  Serial.printf("set_head_pose(pitch=%f, roll=%f)\n", pitch_radians, roll_radians);
  
  SetHeadRollDegrees((roll_radians * 180.0f) / M_PI);
  SetHeadPitchDegrees((pitch_radians * 180.0f) / M_PI);
}
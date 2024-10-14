#ifndef SET_HEAD_POSE_ACTION_HANDLER_
#define SET_HEAD_POSE_ACTION_HANDLER_

#include "p2p_action_server.h"

class SetHeadPoseActionHandler : public P2PActionHandler<P2PSetHeadPoseRequest> {
public:
  // Does not take ownsership of the pointee, which must outlive this object.
  SetHeadPoseActionHandler(P2PPacketStreamArduino *p2p_stream)
    : P2PActionHandler<P2PSetHeadPoseRequest>(P2PAction::kSetHeadPose, p2p_stream) {}

  bool Run() override;
};

#endif  // SET_HEAD_POSE_ACTION_HANDLER_
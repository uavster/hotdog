#ifndef CREATE_HEAD_TRAJECTORY_VIEW_ACTION_HANDLER_
#define CREATE_HEAD_TRAJECTORY_VIEW_ACTION_HANDLER_

#include "p2p_action_server.h"
#include "trajectory_store.h"
#include "logger_interface.h"

class CreateHeadTrajectoryViewActionHandler : public P2PActionHandler<P2PCreateHeadTrajectoryViewRequest, P2PCreateHeadTrajectoryViewReply> {
public:
  // Does not take ownsership of the pointee, which must outlive this object.
  CreateHeadTrajectoryViewActionHandler(P2PPacketStreamArduino *p2p_stream, TrajectoryStore *trajectory_store)
    : P2PActionHandler<P2PCreateHeadTrajectoryViewRequest, P2PCreateHeadTrajectoryViewReply>(P2PAction::kCreateHeadTrajectoryView, p2p_stream), 
      trajectory_store_(*ASSERT_NOT_NULL(trajectory_store)) {}

  bool Run() override;

private:
  bool TrySendingReply();

  TrajectoryStore &trajectory_store_;  
  Status result_;
  enum { kProcessingRequest, kSendingReply } state_ = kProcessingRequest;  
};

#endif  // CREATE_HEAD_TRAJECTORY_VIEW_ACTION_HANDLER_
#ifndef CREATE_BASE_TRAJECTORY_ACTION_HANDLER_
#define CREATE_BASE_TRAJECTORY_ACTION_HANDLER_

#include "p2p_action_server.h"
#include "trajectory_store.h"
#include "logger_interface.h"

class CreateBaseTrajectoryViewActionHandler : public P2PActionHandler<P2PCreateBaseTrajectoryViewRequest, P2PCreateBaseTrajectoryViewReply> {
public:
  // Does not take ownsership of the pointee, which must outlive this object.
  CreateBaseTrajectoryViewActionHandler(P2PPacketStreamArduino *p2p_stream, TrajectoryStore *trajectory_store)
    : P2PActionHandler<P2PCreateBaseTrajectoryViewRequest, P2PCreateBaseTrajectoryViewReply>(P2PAction::kCreateBaseTrajectoryView, p2p_stream), 
      trajectory_store_(*ASSERT_NOT_NULL(trajectory_store)) {}

  bool Run() override;

private:
  bool TrySendingReply();

  TrajectoryStore &trajectory_store_;  
  Status result_;
  enum { kProcessingRequest, kSendingReply } state_ = kProcessingRequest;  
};

#endif  // CREATE_BASE_TRAJECTORY_ACTION_HANDLER_
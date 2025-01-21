#ifndef EXECUTE_HEAD_TRAJECTORY_VIEW_ACTION_HANDLER_
#define EXECUTE_HEAD_TRAJECTORY_VIEW_ACTION_HANDLER_

#include "p2p_action_server.h"
#include "trajectory_store.h"
#include "logger_interface.h"
#include "head_controller.h"

class ExecuteHeadTrajectoryViewActionHandler : public P2PActionHandler<P2PExecuteHeadTrajectoryViewRequest, P2PExecuteHeadTrajectoryViewReply, P2PExecuteHeadTrajectoryViewProgress> {
public:
  // Does not take ownsership of the pointee, which must outlive this object.
  ExecuteHeadTrajectoryViewActionHandler(P2PPacketStreamArduino *p2p_stream, TrajectoryStore *trajectory_store, HeadTrajectoryController *head_trajectory_controller)
    : P2PActionHandler<P2PExecuteHeadTrajectoryViewRequest, P2PExecuteHeadTrajectoryViewReply, P2PExecuteHeadTrajectoryViewProgress>(P2PAction::kExecuteHeadTrajectoryView, p2p_stream), 
      trajectory_store_(*ASSERT_NOT_NULL(trajectory_store)),
      head_trajectory_controller_(*ASSERT_NOT_NULL(head_trajectory_controller)) {}

  bool Run() override;
  void OnCancel() override;

private:
  bool TrySendingReply();
  bool TrySendingProgress();

  TrajectoryStore &trajectory_store_;  
  HeadTrajectoryController &head_trajectory_controller_;
  Status result_;
  uint64_t last_progress_update_ns_;
  enum { kProcessingRequest, kSendingReply, kWaitForNextProgressUpdate, kSendingProgress } state_ = kProcessingRequest;
};

#endif  // EXECUTE_HEAD_TRAJECTORY_VIEW_ACTION_HANDLER_
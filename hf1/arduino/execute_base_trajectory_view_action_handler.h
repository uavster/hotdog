#ifndef EXECUTE_BASE_TRAJECTORY_VIEW_ACTION_HANDLER_
#define EXECUTE_BASE_TRAJECTORY_VIEW_ACTION_HANDLER_

#include "p2p_action_server.h"
#include "trajectory_store.h"
#include "logger_interface.h"
#include "base_controller.h"

class ExecuteBaseTrajectoryViewActionHandler : public P2PActionHandler<P2PExecuteBaseTrajectoryViewRequest, P2PExecuteBaseTrajectoryViewReply, P2PExecuteBaseTrajectoryViewProgress> {
public:
  // Does not take ownsership of the pointee, which must outlive this object.
  ExecuteBaseTrajectoryViewActionHandler(P2PPacketStreamArduino *p2p_stream, TrajectoryStore *trajectory_store, BaseTrajectoryController *base_trajectory_controller)
    : P2PActionHandler<P2PExecuteBaseTrajectoryViewRequest, P2PExecuteBaseTrajectoryViewReply, P2PExecuteBaseTrajectoryViewProgress>(P2PAction::kExecuteBaseTrajectoryView, p2p_stream), 
      trajectory_store_(*ASSERT_NOT_NULL(trajectory_store)),
      base_trajectory_controller_(*ASSERT_NOT_NULL(base_trajectory_controller)) {}

  bool Run() override;
  bool OnRequest() override;
  void OnCancel() override;

private:
  bool TrySendingReply();
  bool TrySendingProgress();

  TrajectoryStore &trajectory_store_;  
  BaseTrajectoryController &base_trajectory_controller_;
  Status result_;
  uint64_t last_progress_update_ns_;
  enum { kProcessingRequest, kSendingReply, kWaitForNextProgressUpdate, kSendingProgress } state_ = kProcessingRequest;
};

#endif  // EXECUTE_BASE_TRAJECTORY_VIEW_ACTION_HANDLER_
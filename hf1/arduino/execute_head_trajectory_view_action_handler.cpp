#include "execute_head_trajectory_view_action_handler.h"
#include "robot_state_estimator.h"
#include "timer.h"
#include "logger_interface.h"

// Throttling rate in maximum number of packets per second.
// This is important to let other packets be sent when IMU polling
// is blocking.
#define kMaxProgressMessagesPerSecond 10   // Must be > 0.

bool ExecuteHeadTrajectoryViewActionHandler::OnRequest() { 
  state_ = kProcessingRequest;
  return true; 
}

bool ExecuteHeadTrajectoryViewActionHandler::Run() {
  switch(state_) {
    case kProcessingRequest: {
      const P2PExecuteHeadTrajectoryViewRequest &request = GetRequest();
      const auto trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view_id.id));
      const auto trajectory_view_type = static_cast<P2PTrajectoryViewType>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view_id.type));
      last_progress_update_ns_ = 0;

      char str[100];
      sprintf(str, "execute_head_trajectory_view(trajectory_view_type=%d, trajectory_view_id=%d)", trajectory_view_type, trajectory_view_id);
      LOG_INFO(str);

      TrajectoryViewInterface<HeadTargetState> *trajectory_view = nullptr;
      switch(trajectory_view_type) {
        case kPlain: {
          auto &maybe_trajectory_view = trajectory_store_.head_trajectory_views()[trajectory_view_id];
          result_ = maybe_trajectory_view.status();
          if (maybe_trajectory_view.ok()) {
            trajectory_view = &*maybe_trajectory_view;
          }
          break;
        }
        case kModulated: {
          auto &maybe_trajectory_view = trajectory_store_.head_modulated_trajectory_views()[trajectory_view_id];
          result_ = maybe_trajectory_view.status();
          if (maybe_trajectory_view.ok()) {
            trajectory_view = &*maybe_trajectory_view;
          }
          break;
        }
        case kMixed: {
          auto &maybe_trajectory_view = trajectory_store_.head_mixed_trajectory_views()[trajectory_view_id];
          result_ = maybe_trajectory_view.status();
          if (maybe_trajectory_view.ok()) {
            trajectory_view = &*maybe_trajectory_view;
          }
          break;
        }
      }

      if (result_ != Status::kSuccess) {
        if (TrySendingReply()) { 
          state_ = kProcessingRequest; // Get ready for the next command.
          return false;
        }
        state_ = kSendingReply;
        break;
      }

      head_trajectory_controller_.trajectory(trajectory_view);
      head_trajectory_controller_.Start();
      state_ = kWaitForNextProgressUpdate;
      break;
    }

    case kWaitForNextProgressUpdate: {
      if (!head_trajectory_controller_.IsTrajectoryFinished() && 
          GetTimerNanoseconds() - last_progress_update_ns_ <= (1'000'000'000ULL / kMaxProgressMessagesPerSecond)) {
        break;
      }
      if (head_trajectory_controller_.IsTrajectoryFinished()) {
        if (TrySendingReply()) {
          state_ = kProcessingRequest; // Get ready for the next command.
          return false; // Do not loop anymore.
        } else {
          state_ = kSendingReply;
        }
      } else {
        if (TrySendingProgress()) {
          state_ = kWaitForNextProgressUpdate;
        } else {
          state_ = kSendingProgress;
        }
      }
      break;    
    }

    case kSendingReply: {
      if (TrySendingReply()) {
        state_ = kProcessingRequest; // Get ready for the next command.
        return false; // Do not loop anymore.
      }
      break;
    }

    case kSendingProgress: {
      if (TrySendingProgress()) {
        state_ = kWaitForNextProgressUpdate;
      }
      break;
    }
  }
  return true;  // Keep looping.
}

bool ExecuteHeadTrajectoryViewActionHandler::TrySendingReply() {
  StatusOr<P2PActionPacketAdapter<P2PExecuteHeadTrajectoryViewReply>> maybe_reply = NewReply();
  if (!maybe_reply.ok()) {
    return false;
  }
  last_progress_update_ns_ = GetTimerNanoseconds();
  P2PActionPacketAdapter<P2PExecuteHeadTrajectoryViewReply> reply = *maybe_reply;
  reply->status_code = LocalToNetwork<kP2PLocalEndianness>(result_);
  reply.Commit(/*guarantee_delivery=*/true);
  return true;
}

bool ExecuteHeadTrajectoryViewActionHandler::TrySendingProgress() {
  StatusOr<P2PActionPacketAdapter<P2PExecuteHeadTrajectoryViewProgress>> maybe_progress = NewProgress();
  if (!maybe_progress.ok()) {
    return false;
  }
  last_progress_update_ns_ = GetTimerNanoseconds();
  P2PActionPacketAdapter<P2PExecuteHeadTrajectoryViewProgress> progress = *maybe_progress;
  progress->num_completed_laps = LocalToNetwork<kP2PLocalEndianness>(head_trajectory_controller_.NumCompletedLaps());
  progress.Commit(/*guarantee_delivery=*/false);
  return true;
}

void ExecuteHeadTrajectoryViewActionHandler::OnCancel() {
  head_trajectory_controller_.Stop();
}
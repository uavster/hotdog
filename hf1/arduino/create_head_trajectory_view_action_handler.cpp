#include "create_head_trajectory_view_action_handler.h"
#include "trajectory_store.h"

bool CreateHeadTrajectoryViewActionHandler::Run() {
  switch(state_) {
    case kProcessingRequest: {
      const P2PCreateHeadTrajectoryViewRequest &request = GetRequest();

      const int trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.id));
      const int trajectory_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.trajectory_id));
      const float loop_after_seconds = NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.loop_after_seconds);
      InterpolationConfig interpolation_config;
      interpolation_config.type = static_cast<InterpolationType>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.interpolation_config.type));

      char str[150];
      sprintf(str, "create_head_trajectory_view(id=%d, trajectory_id=%d, loop_after_seconds=%f, interpolation_config={type=%d})", trajectory_view_id, trajectory_id, loop_after_seconds, interpolation_config.type);
      LOG_INFO(str);

      auto &maybe_trajectory_view = trajectory_store_.head_trajectory_views()[trajectory_view_id];
      if (maybe_trajectory_view.status() == Status::kDoesNotExistError) {
        result_ = maybe_trajectory_view.status();
      } else {
        const auto maybe_trajectory = trajectory_store_.head_trajectories()[trajectory_id];
        if (!maybe_trajectory.ok()) {
          result_ = maybe_trajectory.status();
        } else {
          result_ = Status::kSuccess;
          maybe_trajectory_view = HeadTrajectoryView(&*maybe_trajectory);
          if (loop_after_seconds >= 0) {
            maybe_trajectory_view->EnableLooping(loop_after_seconds);
          } else {
            maybe_trajectory_view->DisableLooping();
          }
          if (interpolation_config.type == InterpolationType::kNone) {
            maybe_trajectory_view->DisableInterpolation();
          } else {
            maybe_trajectory_view->EnableInterpolation(interpolation_config);
          }
        }
      }
      if (TrySendingReply()) {
        return false; // Reply sent; do not call Run() again.
      }
      state_ = kSendingReply;
      break;
    }
    
    case kSendingReply: {
      if (TrySendingReply()) {
        state_ = kProcessingRequest;
        return false; // Reply sent; do not call Run() again.
      }
      break;
    }
  }
  return true;
}

bool CreateHeadTrajectoryViewActionHandler::TrySendingReply() {
  StatusOr<P2PActionPacketAdapter<P2PCreateHeadTrajectoryViewReply>> maybe_reply = NewReply();
  if (!maybe_reply.ok()) {
    return false;
  }
  P2PActionPacketAdapter<P2PCreateHeadTrajectoryViewReply> reply = *maybe_reply;
  reply->status_code = LocalToNetwork<kP2PLocalEndianness>(result_);
  reply.Commit(/*guarantee_delivery=*/true);
  return true;
}

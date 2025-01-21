#include "create_base_mixed_trajectory_view_action_handler.h"
#include "trajectory_store.h"

bool CreateBaseMixedTrajectoryViewActionHandler::Run() {
  switch(state_) {
    case kProcessingRequest: {
      const P2PCreateBaseMixedTrajectoryViewRequest &request = GetRequest();

      const int mixed_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.id));
      const int first_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.first_trajectory_view_id));
      const int second_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.second_trajectory_view_id));
      const int alpha_envelope_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.alpha_envelope_trajectory_view_id));

      char str[200];
      sprintf(str, "create_base_mixed_trajectory_view(id=%d, first_trajectory_view_id=%d, second_trajectory_view_id=%d, alpha_trajectory_view_id=%d)", mixed_trajectory_view_id, first_trajectory_view_id, second_trajectory_view_id, alpha_envelope_trajectory_view_id);
      LOG_INFO(str);

      auto &maybe_mixed_trajectory_view = trajectory_store_.base_mixed_trajectory_views()[mixed_trajectory_view_id];
      if (maybe_mixed_trajectory_view.status() == Status::kDoesNotExistError) {
        result_ = maybe_mixed_trajectory_view.status();
      } else {
        const auto &maybe_first_trajectory_view_id = trajectory_store_.base_trajectory_views()[first_trajectory_view_id];
        if (!maybe_first_trajectory_view_id.ok()) {
          result_ = maybe_first_trajectory_view_id.status();
        } else {
          const auto &maybe_second_trajectory_view_id = trajectory_store_.base_trajectory_views()[second_trajectory_view_id];
          if (!maybe_second_trajectory_view_id.ok()) {
            result_ = maybe_second_trajectory_view_id.status();
          } else {
            const auto &maybe_alpha_envelope_trajectory_view_id = trajectory_store_.envelope_trajectory_views()[alpha_envelope_trajectory_view_id];
            if (!maybe_alpha_envelope_trajectory_view_id.ok()) {
              result_ = maybe_alpha_envelope_trajectory_view_id.status();
            } else {
              result_ = Status::kSuccess;
              maybe_mixed_trajectory_view = BaseMixedTrajectoryView();
              maybe_mixed_trajectory_view->trajectory1(&*maybe_first_trajectory_view_id);
              maybe_mixed_trajectory_view->trajectory2(&*maybe_second_trajectory_view_id);
              maybe_mixed_trajectory_view->alpha(&*maybe_alpha_envelope_trajectory_view_id);
            }
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

bool CreateBaseMixedTrajectoryViewActionHandler::TrySendingReply() {
  StatusOr<P2PActionPacketAdapter<P2PCreateBaseMixedTrajectoryViewReply>> maybe_reply = NewReply();
  if (!maybe_reply.ok()) {
    return false;
  }
  P2PActionPacketAdapter<P2PCreateBaseMixedTrajectoryViewReply> reply = *maybe_reply;
  reply->status_code = LocalToNetwork<kP2PLocalEndianness>(result_);
  reply.Commit(/*guarantee_delivery=*/true);
  return true;
}

#include "create_base_modulated_trajectory_view_action_handler.h"
#include "trajectory_store.h"

bool CreateBaseModulatedTrajectoryViewActionHandler::Run() {
  switch(state_) {
    case kProcessingRequest: {
      const P2PCreateBaseModulatedTrajectoryViewRequest &request = GetRequest();

      const int modulated_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.id));
      const int carrier_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.carrier_trajectory_view_id));
      const int modulator_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.modulator_trajectory_view_id));
      const int envelope_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.envelope_trajectory_view_id));

      char str[200];
      sprintf(str, "create_base_modulated_trajectory_view(id=%d, carrier_trajectory_view_id=%d, modulator_trajectory_view_id=%d, envelope_trajectory_view_id=%d)", modulated_trajectory_view_id, carrier_trajectory_view_id, modulator_trajectory_view_id, envelope_trajectory_view_id);
      LOG_INFO(str);

      auto &maybe_modulated_trajectory_view = trajectory_store_.base_modulated_trajectory_views()[modulated_trajectory_view_id];
      if (maybe_modulated_trajectory_view.status() == Status::kDoesNotExistError) {
        result_ = maybe_modulated_trajectory_view.status();
      } else {
        const auto maybe_carrier_trajectory_view_id = trajectory_store_.base_trajectory_views()[carrier_trajectory_view_id];
        if (!maybe_carrier_trajectory_view_id.ok()) {
          result_ = maybe_carrier_trajectory_view_id.status();
        } else {
          const auto maybe_modulator_trajectory_view_id = trajectory_store_.base_trajectory_views()[modulator_trajectory_view_id];
          if (!maybe_modulator_trajectory_view_id.ok()) {
            result_ = maybe_modulator_trajectory_view_id.status();
          } else {
            const auto maybe_envelope_trajectory_view_id = trajectory_store_.envelope_trajectory_views()[envelope_trajectory_view_id];
            if (!maybe_envelope_trajectory_view_id.ok()) {
              result_ = maybe_envelope_trajectory_view_id.status();
            } else {
              result_ = Status::kSuccess;
              maybe_modulated_trajectory_view = BaseModulatedTrajectoryView();
              maybe_modulated_trajectory_view->carrier(&*maybe_carrier_trajectory_view_id);
              maybe_modulated_trajectory_view->modulator(&*maybe_modulator_trajectory_view_id);
              maybe_modulated_trajectory_view->envelope(&*maybe_envelope_trajectory_view_id);
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

bool CreateBaseModulatedTrajectoryViewActionHandler::TrySendingReply() {
  StatusOr<P2PActionPacketAdapter<P2PCreateBaseModulatedTrajectoryViewReply>> maybe_reply = NewReply();
  if (!maybe_reply.ok()) {
    return false;
  }
  P2PActionPacketAdapter<P2PCreateBaseModulatedTrajectoryViewReply> reply = *maybe_reply;
  reply->status_code = LocalToNetwork<kP2PLocalEndianness>(result_);
  reply.Commit(/*guarantee_delivery=*/true);
  return true;
}

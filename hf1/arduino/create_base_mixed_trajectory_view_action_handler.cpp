#include "create_base_mixed_trajectory_view_action_handler.h"
#include "trajectory_store.h"

bool CreateBaseMixedTrajectoryViewActionHandler::Run() {
  switch(state_) {
    case kProcessingRequest: {
      const P2PCreateBaseMixedTrajectoryViewRequest &request = GetRequest();

      const int mixed_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.id));
      const auto first_trajectory_view_type = static_cast<P2PTrajectoryViewType>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.first_trajectory_view_id.type));
      const auto first_trajectory_view_id = NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.first_trajectory_view_id.id);
      const auto second_trajectory_view_type = static_cast<P2PTrajectoryViewType>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.second_trajectory_view_id.type));
      const auto second_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.second_trajectory_view_id.id));
      const int alpha_envelope_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.alpha_envelope_trajectory_view_id));

      char str[220];
      sprintf(str, "create_base_mixed_trajectory_view(id=%d, first_trajectory_view_id=%s:%d, second_trajectory_view_id=%s:%d, alpha_trajectory_view_id=%d)", mixed_trajectory_view_id, GetTrajectoryViewTypeName(first_trajectory_view_type), first_trajectory_view_id, GetTrajectoryViewTypeName(second_trajectory_view_type), second_trajectory_view_id, alpha_envelope_trajectory_view_id);
      LOG_INFO(str);

      auto &maybe_mixed_trajectory_view = trajectory_store_.base_mixed_trajectory_views()[mixed_trajectory_view_id];
      if (maybe_mixed_trajectory_view.status() == Status::kDoesNotExistError) {
        result_ = maybe_mixed_trajectory_view.status();
        LOG_ERROR("Index of mixed trajectory view is out of bounds.");
      } else {
        result_ = Status::kSuccess;
        const TrajectoryViewInterface<BaseTargetState> *trajectory1_view = nullptr;
        const TrajectoryViewInterface<BaseTargetState> *trajectory2_view = nullptr;
        const EnvelopeTrajectoryView *alpha_view = nullptr;

        switch(first_trajectory_view_type) {
          case P2PTrajectoryViewType::kPlain: {
            const auto &maybe_first_trajectory_view = trajectory_store_.base_trajectory_views()[first_trajectory_view_id];
            if (!maybe_first_trajectory_view.ok()) {
              result_ = maybe_first_trajectory_view.status();
              LOG_ERROR("The first plain trajectory view does not exist.");
            } else {
              trajectory1_view = &*maybe_first_trajectory_view;
            }
            break;
          }
          case P2PTrajectoryViewType::kModulated: {
            const auto &maybe_first_trajectory_view = trajectory_store_.base_modulated_trajectory_views()[first_trajectory_view_id];
            if (!maybe_first_trajectory_view.ok()) {
              result_ = maybe_first_trajectory_view.status();
              LOG_ERROR("The first modulated trajectory view does not exist.");
            } else {
              trajectory1_view = &*maybe_first_trajectory_view;
            }
            break;
          }
          case P2PTrajectoryViewType::kMixed: {
            const auto &maybe_first_trajectory_view = trajectory_store_.base_mixed_trajectory_views()[first_trajectory_view_id];
            if (!maybe_first_trajectory_view.ok()) {
              result_ = maybe_first_trajectory_view.status();
              LOG_ERROR("The first mixed view does not exist.");
            } else {
              trajectory1_view = &*maybe_first_trajectory_view;
            }
            break;
          }
          default:
            LOG_ERROR("Invalid type for first trajectory view.");
            result_ = Status::kMalformedError;
            break;
        }

        if (result_ == Status::kSuccess) {
          switch(second_trajectory_view_type) {
            case P2PTrajectoryViewType::kPlain: {
              const auto &maybe_second_trajectory_view = trajectory_store_.base_trajectory_views()[second_trajectory_view_id];
              if (!maybe_second_trajectory_view.ok()) {
                result_ = maybe_second_trajectory_view.status();
                LOG_ERROR("The second plain trajectory view does not exist.");
              } else {
                trajectory2_view = &*maybe_second_trajectory_view;
              }
              break;
            }
            case P2PTrajectoryViewType::kModulated: {
              const auto &maybe_second_trajectory_view = trajectory_store_.base_modulated_trajectory_views()[second_trajectory_view_id];
              if (!maybe_second_trajectory_view.ok()) {
                result_ = maybe_second_trajectory_view.status();
                LOG_ERROR("The second modulated trajectory view does not exist.");
              } else {
                trajectory2_view = &*maybe_second_trajectory_view;
              }
              break;
            }
            case P2PTrajectoryViewType::kMixed: {
              const auto &maybe_second_trajectory_view = trajectory_store_.base_mixed_trajectory_views()[second_trajectory_view_id];
              if (!maybe_second_trajectory_view.ok()) {
                result_ = maybe_second_trajectory_view.status();
                LOG_ERROR("The second mixed trajectory view does not exist.");
              } else {
                trajectory2_view = &*maybe_second_trajectory_view;
              }
              break;
            }
            default:
              LOG_ERROR("Invalid type for second trajectory view.");
              result_ = Status::kMalformedError;
              break;
          }
        }

        if (result_ == Status::kSuccess) {
          const auto maybe_alpha_envelope_trajectory_view = trajectory_store_.envelope_trajectory_views()[alpha_envelope_trajectory_view_id];
          if (!maybe_alpha_envelope_trajectory_view.ok()) {
            result_ = maybe_alpha_envelope_trajectory_view.status();
            LOG_ERROR("The alpha trajectory view does not exist.");
          } else {
            alpha_view = &*maybe_alpha_envelope_trajectory_view;
          }
        }

        if (result_ == Status::kSuccess) {
          result_ = Status::kSuccess;
          maybe_mixed_trajectory_view = BaseMixedTrajectoryView();
          maybe_mixed_trajectory_view->trajectory1(trajectory1_view);
          maybe_mixed_trajectory_view->trajectory2(trajectory2_view);
          maybe_mixed_trajectory_view->alpha(alpha_view);
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

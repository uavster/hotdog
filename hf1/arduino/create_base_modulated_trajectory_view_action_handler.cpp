#include "create_base_modulated_trajectory_view_action_handler.h"
#include "trajectory_store.h"

bool CreateBaseModulatedTrajectoryViewActionHandler::Run() {
  switch(state_) {
    case kProcessingRequest: {
      const P2PCreateBaseModulatedTrajectoryViewRequest &request = GetRequest();

      const int modulated_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.id));
      const auto carrier_trajectory_view_type = static_cast<P2PTrajectoryViewType>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.carrier_trajectory_view_id.type));
      const auto carrier_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.carrier_trajectory_view_id.id));
      const auto modulator_trajectory_view_type = static_cast<P2PTrajectoryViewType>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.modulator_trajectory_view_id.type));
      const auto modulator_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.modulator_trajectory_view_id.id));
      const auto envelope_trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view.envelope_trajectory_view_id));

      char str[220];
      sprintf(str, "create_base_modulated_trajectory_view(id=%d, carrier_trajectory_view_id=%s:%d, modulator_trajectory_view_id=%s:%d, envelope_trajectory_view_id=%d)", modulated_trajectory_view_id, GetTrajectoryViewTypeName(carrier_trajectory_view_type), carrier_trajectory_view_id, GetTrajectoryViewTypeName(modulator_trajectory_view_type), modulator_trajectory_view_id, envelope_trajectory_view_id);
      LOG_INFO(str);

      auto &maybe_modulated_trajectory_view = trajectory_store_.base_modulated_trajectory_views()[modulated_trajectory_view_id];
      if (maybe_modulated_trajectory_view.status() == Status::kDoesNotExistError) {
        result_ = maybe_modulated_trajectory_view.status();
        LOG_ERROR("Index of modulated trajectory view is out of bounds.");
      } else {
        result_ = Status::kSuccess;
        const TrajectoryViewInterface<BaseTargetState> *carrier_view = nullptr;
        const TrajectoryViewInterface<BaseTargetState> *modulator_view = nullptr;
        const EnvelopeTrajectoryView *envelope_view = nullptr;

        switch(carrier_trajectory_view_type) {
          case P2PTrajectoryViewType::kPlain: {
            const auto &maybe_carrier_trajectory_view = trajectory_store_.base_trajectory_views()[carrier_trajectory_view_id];
            if (!maybe_carrier_trajectory_view.ok()) {
              result_ = maybe_carrier_trajectory_view.status();
              LOG_ERROR("The carrier plain trajectory view does not exist.");
            } else {
              carrier_view = &*maybe_carrier_trajectory_view;
            }
            break;
          }
          case P2PTrajectoryViewType::kModulated: {
            const auto &maybe_carrier_trajectory_view = trajectory_store_.base_modulated_trajectory_views()[carrier_trajectory_view_id];
            if (!maybe_carrier_trajectory_view.ok()) {
              result_ = maybe_carrier_trajectory_view.status();
              LOG_ERROR("The carrier modulated trajectory view does not exist.");
            } else {
              carrier_view = &*maybe_carrier_trajectory_view;
            }
            break;
          }
          case P2PTrajectoryViewType::kMixed: {
            const auto &maybe_carrier_trajectory_view = trajectory_store_.base_mixed_trajectory_views()[carrier_trajectory_view_id];
            if (!maybe_carrier_trajectory_view.ok()) {
              result_ = maybe_carrier_trajectory_view.status();
              LOG_ERROR("The carrier mixed trajectory view does not exist.");
            } else {
              carrier_view = &*maybe_carrier_trajectory_view;
            }
            break;
          }
          default:
            LOG_ERROR("Invalid type for carrier trajectory view.");
            result_ = Status::kMalformedError;
            break;
        }

        if (result_ == Status::kSuccess) {
          switch(modulator_trajectory_view_type) {
            case P2PTrajectoryViewType::kPlain: {
              const auto &maybe_second_trajectory_view = trajectory_store_.base_trajectory_views()[modulator_trajectory_view_id];
              if (!maybe_second_trajectory_view.ok()) {
                result_ = maybe_second_trajectory_view.status();
                LOG_ERROR("The modulator plain trajectory view does not exist.");
              } else {
                modulator_view = &*maybe_second_trajectory_view;
              }
              break;
            }
            case P2PTrajectoryViewType::kModulated: {
              const auto &maybe_second_trajectory_view = trajectory_store_.base_modulated_trajectory_views()[modulator_trajectory_view_id];
              if (!maybe_second_trajectory_view.ok()) {
                result_ = maybe_second_trajectory_view.status();
                LOG_ERROR("The modulator modulated trajectory view does not exist.");
              } else {
                modulator_view = &*maybe_second_trajectory_view;
              }
              break;
            }
            case P2PTrajectoryViewType::kMixed: {
              const auto &maybe_second_trajectory_view = trajectory_store_.base_mixed_trajectory_views()[modulator_trajectory_view_id];
              if (!maybe_second_trajectory_view.ok()) {
                result_ = maybe_second_trajectory_view.status();
                LOG_ERROR("The modulator mixed trajectory view does not exist.");
              } else {
                modulator_view = &*maybe_second_trajectory_view;
              }
              break;
            }
            default:
              LOG_ERROR("Invalid type for modulator trajectory view.");
              result_ = Status::kMalformedError;
              break;
          }
        }

        if (result_ == Status::kSuccess) {
          const auto maybe_envelope_trajectory_view = trajectory_store_.envelope_trajectory_views()[envelope_trajectory_view_id];
          if (!maybe_envelope_trajectory_view.ok()) {
            result_ = maybe_envelope_trajectory_view.status();
            LOG_ERROR("The envelope trajectory view does not exist.");
          } else {
            envelope_view = &*maybe_envelope_trajectory_view;
          }
        }

        if (result_ == Status::kSuccess) {
          result_ = Status::kSuccess;
          maybe_modulated_trajectory_view = BaseModulatedTrajectoryView();
          maybe_modulated_trajectory_view->carrier(carrier_view);
          maybe_modulated_trajectory_view->modulator(modulator_view);
          maybe_modulated_trajectory_view->envelope(envelope_view);
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

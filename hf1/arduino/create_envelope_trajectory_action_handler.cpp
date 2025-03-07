#include "create_envelope_trajectory_action_handler.h"
#include "trajectory_store.h"

bool CreateEnvelopeTrajectoryActionHandler::Run() {
  switch(state_) {
    case kProcessingRequest: {
      const P2PCreateEnvelopeTrajectoryRequest &request = GetRequest();

      const int trajectory_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.id));
      const int num_waypoints = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory.num_waypoints));
      
      char str[80];
      sprintf(str, "create_envelope_trajectory(id=%d, num_waypoints=%d)", trajectory_id, num_waypoints);
      LOG_INFO(str);

      auto &maybe_trajectory = trajectory_store_.envelope_trajectories()[trajectory_id];
      if (maybe_trajectory.status() == Status::kDoesNotExistError) {
        result_ = maybe_trajectory.status();
      } else {
        result_ = Status::kSuccess;
        maybe_trajectory = Trajectory<EnvelopeTargetState, kP2PMaxNumWaypointsPerEnvelopeTrajectory>();
        for (int i = 0; i < num_waypoints; ++i) {
          const auto waypoint_seconds = NetworkToLocal<kP2PLocalEndianness>(request.trajectory.waypoints[i].seconds);
          const auto &target_state_msg = request.trajectory.waypoints[i].target_state.location;
          const EnvelopeTargetState target_state({
            EnvelopeStateVars{
              NetworkToLocal<kP2PLocalEndianness>(target_state_msg.value)
            }
          });
          maybe_trajectory->Insert(EnvelopeWaypoint(waypoint_seconds, target_state));
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

bool CreateEnvelopeTrajectoryActionHandler::TrySendingReply() {
  StatusOr<P2PActionPacketAdapter<P2PCreateEnvelopeTrajectoryReply>> maybe_reply = NewReply();
  if (!maybe_reply.ok()) {
    return false;
  }
  P2PActionPacketAdapter<P2PCreateEnvelopeTrajectoryReply> reply = *maybe_reply;
  reply->status_code = LocalToNetwork<kP2PLocalEndianness>(result_);
  reply.Commit(/*guarantee_delivery=*/true);
  return true;
}

#include "create_head_trajectory_action_handler.h"
#include "trajectory_store.h"

bool CreateHeadTrajectoryActionHandler::Run() {
  switch(state_) {
    case kProcessingRequest: {
      const P2PCreateHeadTrajectoryRequest &request = GetRequest();

      const int trajectory_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.id));
      const int num_waypoints = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory.num_waypoints));
      
      char str[80];
      sprintf(str, "create_head_trajectory(id=%d, num_waypoints=%d)", trajectory_id, num_waypoints);
      LOG_INFO(str);

      auto &maybe_trajectory = trajectory_store_.head_trajectories()[trajectory_id];
      if (maybe_trajectory.status() == Status::kDoesNotExistError) {
        result_ = maybe_trajectory.status();
      } else {
        result_ = Status::kSuccess;
        maybe_trajectory = Trajectory<HeadTargetState, kP2PMaxNumWaypointsPerHeadTrajectory>();
        for (int i = 0; i < num_waypoints; ++i) {
          const auto waypoint_seconds = NetworkToLocal<kP2PLocalEndianness>(request.trajectory.waypoints[i].seconds);
          const auto &target_state_msg = request.trajectory.waypoints[i].target_state.location;
          const HeadTargetState target_state({
            HeadStateVars{
              NetworkToLocal<kP2PLocalEndianness>(target_state_msg.yaw_radians), 
              NetworkToLocal<kP2PLocalEndianness>(target_state_msg.pitch_radians), 
              NetworkToLocal<kP2PLocalEndianness>(target_state_msg.roll_radians)
            }
          });
          maybe_trajectory->Insert(HeadWaypoint(waypoint_seconds, target_state));
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

bool CreateHeadTrajectoryActionHandler::TrySendingReply() {
  StatusOr<P2PActionPacketAdapter<P2PCreateHeadTrajectoryReply>> maybe_reply = NewReply();
  if (!maybe_reply.ok()) {
    return false;
  }
  P2PActionPacketAdapter<P2PCreateHeadTrajectoryReply> reply = *maybe_reply;
  reply->status_code = LocalToNetwork<kP2PLocalEndianness>(result_);
  reply.Commit(/*guarantee_delivery=*/true);
  return true;
}

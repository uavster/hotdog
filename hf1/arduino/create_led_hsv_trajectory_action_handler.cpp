#include "create_led_hsv_trajectory_action_handler.h"
#include "trajectory_store.h"
#include <cstdio>

bool CreateLedHSVTrajectoryActionHandler::Run() {
  switch (state_) {
    case kProcessingRequest:
      {
        const P2PCreateLedHSVTrajectoryRequest &request = GetRequest();

        const int trajectory_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.id));
        const int num_waypoints = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory.num_waypoints));

        char str[80];
        sprintf(str, "create_led_hsv_trajectory(id=%d, num_waypoints=%d)", trajectory_id, num_waypoints);
        LOG_INFO(str);

        auto &maybe_trajectory = trajectory_store_.led_hsv_trajectories()[trajectory_id];
        if (maybe_trajectory.status() == Status::kDoesNotExistError) {
          result_ = maybe_trajectory.status();
        } else {
          result_ = Status::kSuccess;
          maybe_trajectory = Trajectory<ColorHSVTargetState, kP2PMaxNumWaypointsPerLedHSVTrajectory>();
          for (int i = 0; i < num_waypoints; ++i) {
            const auto waypoint_seconds = NetworkToLocal<kP2PLocalEndianness>(request.trajectory.waypoints[i].seconds);
            const auto &target_state_msg = request.trajectory.waypoints[i].target_state.location;
            const ColorHSVTargetState target_state({ ColorHSVStateVars{
              ColorHSV(
                NetworkToLocal<kP2PLocalEndianness>(target_state_msg.hue),
                NetworkToLocal<kP2PLocalEndianness>(target_state_msg.saturation),
                NetworkToLocal<kP2PLocalEndianness>(target_state_msg.value)) } });
            maybe_trajectory->Insert(LedHSVWaypoint(waypoint_seconds, target_state));
          }
        }
        if (TrySendingReply()) {
          return false;  // Reply sent; do not call Run() again.
        }
        state_ = kSendingReply;
        break;
      }

    case kSendingReply:
      {
        if (TrySendingReply()) {
          state_ = kProcessingRequest;
          return false;  // Reply sent; do not call Run() again.
        }
        break;
      }
  }
  return true;
}

bool CreateLedHSVTrajectoryActionHandler::TrySendingReply() {
  StatusOr<P2PActionPacketAdapter<P2PCreateLedHSVTrajectoryReply>> maybe_reply = NewReply();
  if (!maybe_reply.ok()) {
    return false;
  }
  P2PActionPacketAdapter<P2PCreateLedHSVTrajectoryReply> reply = *maybe_reply;
  reply->status_code = LocalToNetwork<kP2PLocalEndianness>(result_);
  reply.Commit(/*guarantee_delivery=*/true);
  return true;
}

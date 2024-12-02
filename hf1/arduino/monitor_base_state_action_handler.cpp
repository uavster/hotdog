#include "monitor_base_state_action_handler.h"
#include "robot_state_estimator.h"
#include "timer.h"
#include "logger_interface.h"

// Throttling rate in maximum number of packets per second.
// This is important to let other packets be sent when IMU polling
// is blocking.
#define kMaxProgressMessagesPerSecond 10   // Must be > 0.

bool MonitorBaseStateActionHandler::Run() {
  switch(state_) {
    case kReceiveRequest: {
      const P2PMonitorBaseStateRequest &request = GetRequest();
      num_remaining_messages_ = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.max_updates));
      
      char str[48];
      sprintf(str, "monitor_base_state(max_updates=%d)", num_remaining_messages_);
      LOG_INFO(str);
      if (num_remaining_messages_ == 1) {
        if (TrySendingReply()) { 
          --num_remaining_messages_;
          state_ = kReceiveRequest; // Get ready for the next command.
          return false; // Do not loop anymore.
        } else {
          state_ = kSendingReply;
        }
      } else {
        if (TrySendingProgress()) {
          --num_remaining_messages_;
          state_ = kWaitForNextBaseState;
        } else {
          state_ = kSendingProgress;
        }
      }
      break;
    }

    case kWaitForNextBaseState: {
      if (GetBaseStateUpdateNanos() - last_state_update_ns_ <= (1'000'000'000ULL / kMaxProgressMessagesPerSecond)) {
        break;
      }
      if (num_remaining_messages_ == 1) {
        if (TrySendingReply()) {
          --num_remaining_messages_;
          state_ = kReceiveRequest; // Get ready for the next command.
          return false; // Do not loop anymore.
        } else {
          state_ = kSendingReply;
        }
      } else {
        if (TrySendingProgress()) {
          --num_remaining_messages_;
          state_ = kWaitForNextBaseState;
        } else {
          state_ = kSendingProgress;
        }
      }
      break;    
    }

    case kSendingReply: {
      if (TrySendingReply()) {
        --num_remaining_messages_;
        state_ = kReceiveRequest; // Get ready for the next command.
        return false; // Do not loop anymore.
      }
      break;
    }

    case kSendingProgress: {
      if (TrySendingProgress()) {
        --num_remaining_messages_;
        state_ = kWaitForNextBaseState;
      }
      break;
    }
  }
  return true;  // Keep looping.
}

bool MonitorBaseStateActionHandler::TrySendingReply() {
  const BaseState base_state = GetBaseState();
  StatusOr<P2PActionPacketAdapter<P2PMonitorBaseStateReply>> maybe_reply = NewReply();
  if (!maybe_reply.ok()) {
    return false;
  }
  last_state_update_ns_ = GetBaseStateUpdateNanos();
  P2PActionPacketAdapter<P2PMonitorBaseStateReply> reply = *maybe_reply;
  reply->global_timestamp_ns = LocalToNetwork<kP2PLocalEndianness>(system_timer_.GetGlobalNanoseconds());
  reply->position_x = LocalToNetwork<kP2PLocalEndianness>(base_state.location().position().x);
  reply->position_y = LocalToNetwork<kP2PLocalEndianness>(base_state.location().position().y);
  reply->yaw = LocalToNetwork<kP2PLocalEndianness>(base_state.location().yaw());
  reply.Commit(/*guarantee_delivery=*/true);
  return true;
}

bool MonitorBaseStateActionHandler::TrySendingProgress() {
  const BaseState base_state = GetBaseState();
  StatusOr<P2PActionPacketAdapter<P2PMonitorBaseStateProgress>> maybe_progress = NewProgress();
  if (!maybe_progress.ok()) {
    return false;
  }
  last_state_update_ns_ = GetBaseStateUpdateNanos();
  P2PActionPacketAdapter<P2PMonitorBaseStateProgress> progress = *maybe_progress;
  progress->global_timestamp_ns = LocalToNetwork<kP2PLocalEndianness>(system_timer_.GetGlobalNanoseconds());
  progress->position_x = LocalToNetwork<kP2PLocalEndianness>(base_state.location().position().x);
  progress->position_y = LocalToNetwork<kP2PLocalEndianness>(base_state.location().position().y);
  progress->yaw = LocalToNetwork<kP2PLocalEndianness>(base_state.location().yaw());
  progress.Commit(/*guarantee_delivery=*/false);
  return true;
}

#include "execute_led_hsv_trajectory_view_action_handler.h"
#include "timer.h"
#include "logger_interface.h"
#include <cstdio>

// Throttling rate in maximum number of packets per second.
// This is important to let other packets be sent when IMU polling
// is blocking.
#define kMaxProgressMessagesPerSecond 10   // Must be > 0.

bool ExecuteLedHSVTrajectoryViewActionHandler::OnRequest() { 
  state_ = kProcessingRequest;
  return true; 
}

bool ExecuteLedHSVTrajectoryViewActionHandler::Run() {
  switch(state_) {
    case kProcessingRequest: {
      const P2PExecuteLedHSVTrajectoryViewRequest &request = GetRequest();
      const auto trajectory_view_id = static_cast<int>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view_id.id));
      const auto trajectory_view_type = static_cast<P2PTrajectoryViewType>(NetworkToLocal<kP2PLocalEndianness>(request.trajectory_view_id.type));
      last_progress_update_ns_ = 0;

      if (is_enabled_) {
        char str[100];
        sprintf(str, "execute_led_hsv_trajectory_view(trajectory_view_type=%d, trajectory_view_id=%d)", trajectory_view_type, trajectory_view_id);
        LOG_INFO(str);
      } else {
        // The action is disabled: do not run it and send back an abort notification.
        LOG_WARNING("execute_led_hsv_trajectory_view() rejected because it's disabled.");
        state_ = kSendingAbort;
        break;
      }

      TrajectoryViewInterface<ColorHSVTargetState> *trajectory_view = nullptr;
      switch(trajectory_view_type) {
        case kPlain: {
          auto &maybe_trajectory_view = trajectory_store_.led_hsv_trajectory_views()[trajectory_view_id];
          result_ = maybe_trajectory_view.status();
          if (maybe_trajectory_view.ok()) {
            trajectory_view = &*maybe_trajectory_view;
          }
          break;
        }
        case kModulated: {
          auto &maybe_trajectory_view = trajectory_store_.led_hsv_modulated_trajectory_views()[trajectory_view_id];
          result_ = maybe_trajectory_view.status();
          if (maybe_trajectory_view.ok()) {
            trajectory_view = &*maybe_trajectory_view;
          }
          break;
        }
        case kMixed: {
          auto &maybe_trajectory_view = trajectory_store_.led_hsv_mixed_trajectory_views()[trajectory_view_id];
          result_ = maybe_trajectory_view.status();
          if (maybe_trajectory_view.ok()) {
            trajectory_view = &*maybe_trajectory_view;
          }
          break;
        }
        default:
          LOG_ERROR("Invalid trajectory type.");
          result_ = Status::kMalformedError;
          break;
      }

      if (result_ != Status::kSuccess) {
        if (TrySendingReply()) { 
          state_ = kProcessingRequest; // Get ready for the next command.
          return false;
        }
        state_ = kSendingReply;
        break;
      }

      led_hsv_trajectory_controller_.trajectory(trajectory_view);
      led_hsv_trajectory_controller_.Start();
      state_ = kWaitForNextProgressUpdate;
      break;
    }

    case kWaitForNextProgressUpdate: {
      if (!led_hsv_trajectory_controller_.IsTrajectoryFinished() && 
          GetTimerNanoseconds() - last_progress_update_ns_ <= (1'000'000'000ULL / kMaxProgressMessagesPerSecond)) {
        break;
      }
      if (led_hsv_trajectory_controller_.IsTrajectoryFinished()) {
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

    case kSendingAbort: {
      if (TrySendingAbort()) {
        state_ = kProcessingRequest;
        return false; // Do not loop anymore.
      }
    }
  }
  return true;  // Keep looping.
}

void ExecuteLedHSVTrajectoryViewActionHandler::Abort() {
  if (run_state() != kRunning) {
    return;
  }
  led_hsv_trajectory_controller_.Stop();
  led_ui_.Restore();
  state_ = kSendingAbort;
}

bool ExecuteLedHSVTrajectoryViewActionHandler::TrySendingReply() {
  StatusOr<P2PActionPacketAdapter<P2PExecuteLedHSVTrajectoryViewReply>> maybe_reply = NewReply();
  if (!maybe_reply.ok()) {
    return false;
  }
  last_progress_update_ns_ = GetTimerNanoseconds();
  P2PActionPacketAdapter<P2PExecuteLedHSVTrajectoryViewReply> reply = *maybe_reply;
  reply->status_code = LocalToNetwork<kP2PLocalEndianness>(result_);
  reply.Commit(/*guarantee_delivery=*/true);
  return true;
}

bool ExecuteLedHSVTrajectoryViewActionHandler::TrySendingProgress() {
  StatusOr<P2PActionPacketAdapter<P2PExecuteLedHSVTrajectoryViewProgress>> maybe_progress = NewProgress();
  if (!maybe_progress.ok()) {
    return false;
  }
  last_progress_update_ns_ = GetTimerNanoseconds();
  P2PActionPacketAdapter<P2PExecuteLedHSVTrajectoryViewProgress> progress = *maybe_progress;
  progress->num_completed_laps = LocalToNetwork<kP2PLocalEndianness>(led_hsv_trajectory_controller_.NumCompletedLaps());
  progress.Commit(/*guarantee_delivery=*/false);
  return true;
}

bool ExecuteLedHSVTrajectoryViewActionHandler::TrySendingAbort() {
  StatusOr<P2PActionPacketAdapter<P2PExecuteLedHSVTrajectoryViewReply>> maybe_reply = NewAbort();
  if (!maybe_reply.ok()) {
    return false;
  }
  last_progress_update_ns_ = GetTimerNanoseconds();
  P2PActionPacketAdapter<P2PExecuteLedHSVTrajectoryViewReply> reply = *maybe_reply;
  reply->status_code = LocalToNetwork<kP2PLocalEndianness>(Status::kAbortedError);
  reply.Commit(/*guarantee_delivery=*/true);
  return true;
}

void ExecuteLedHSVTrajectoryViewActionHandler::OnCancel() {
  led_hsv_trajectory_controller_.Stop();
  led_ui_.Restore();
}
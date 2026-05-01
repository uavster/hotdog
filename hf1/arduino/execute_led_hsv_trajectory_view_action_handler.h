#ifndef EXECUTE_LEDHSV_TRAJECTORY_VIEW_ACTION_HANDLER_
#define EXECUTE_LEDHSV_TRAJECTORY_VIEW_ACTION_HANDLER_

#include "p2p_action_server.h"
#include "trajectory_store.h"
#include "logger_interface.h"
#include "led_controller.h"
#include "led_ui.h"

class ExecuteLedHSVTrajectoryViewActionHandler : public P2PActionHandler<P2PExecuteLedHSVTrajectoryViewRequest, P2PExecuteLedHSVTrajectoryViewReply, P2PExecuteLedHSVTrajectoryViewProgress> {
public:
  // Does not take ownsership of the pointees, which must outlive this object.
  ExecuteLedHSVTrajectoryViewActionHandler(P2PPacketStreamArduino *p2p_stream, TrajectoryStore *trajectory_store, LedHSVTrajectoryController *led_hsv_trajectory_controller, LedUI *led_ui)
    : P2PActionHandler<P2PExecuteLedHSVTrajectoryViewRequest, P2PExecuteLedHSVTrajectoryViewReply, P2PExecuteLedHSVTrajectoryViewProgress>(P2PAction::kExecuteLedHSVTrajectoryView, p2p_stream), 
      trajectory_store_(*ASSERT_NOT_NULL(trajectory_store)),
      led_hsv_trajectory_controller_(*ASSERT_NOT_NULL(led_hsv_trajectory_controller)),
      led_ui_(*ASSERT_NOT_NULL(led_ui)),
      is_enabled_(true) {}

  bool Run() override;
  void Abort() override;

  bool OnRequest() override;
  void OnCancel() override;

  // If `ie` is true, the action handler processes requests.
  // Otherwise, requests are rejected with an abort packet.
  void is_enabled(bool ie) { is_enabled_ = ie; }
  bool is_enabled() const { return is_enabled_; }

private:
  bool TrySendingReply();
  bool TrySendingProgress();
  bool TrySendingAbort();

  TrajectoryStore &trajectory_store_;  
  LedHSVTrajectoryController &led_hsv_trajectory_controller_;
  LedUI &led_ui_;
  bool is_enabled_;
  Status result_;
  uint64_t last_progress_update_ns_;
  enum { kProcessingRequest, kSendingReply, kWaitForNextProgressUpdate, kSendingProgress, kSendingAbort } state_ = kProcessingRequest;
};

#endif  // EXECUTE_LEDHSV_TRAJECTORY_VIEW_ACTION_HANDLER_
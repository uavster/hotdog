#ifndef SET_BASE_VELOCITY_ACTION_HANDLER_
#define SET_BASE_VELOCITY_ACTION_HANDLER_

#include "p2p_action_server.h"
#include "base_controller.h"
#include "logger_interface.h"

class SetBaseVelocityActionHandler : public P2PActionHandler<P2PSetBaseVelocityRequest> {
public:
  // Does not take ownsership of the pointees, which must outlive this object.
  SetBaseVelocityActionHandler(P2PPacketStreamArduino *p2p_stream, BaseSpeedController *base_speed_controller)
    : P2PActionHandler<P2PSetBaseVelocityRequest>(P2PAction::kSetBaseVelocity, p2p_stream), 
      base_speed_controller_(*ASSERT_NOT_NULL(base_speed_controller)) {}

  bool Run() override;

private:
  BaseSpeedController &base_speed_controller_;
};

#endif  // SET_BASE_VELOCITY_ACTION_HANDLER_
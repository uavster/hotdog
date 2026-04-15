#ifndef PING_ACTION_HANDLER_
#define PING_ACTION_HANDLER_

#include "p2p_action_server.h"

class PingActionHandler : public P2PActionHandler<P2PPingRequest, P2PPingReply> {
public:
  using PingReceivedCallbackType = void (*)();
  // Does not take ownsership of the pointee, which must outlive this object.
  PingActionHandler(P2PPacketStreamArduino *p2p_stream, PingReceivedCallbackType ping_received_callback)
    : P2PActionHandler<P2PPingRequest, P2PPingReply>(P2PAction::kPing, p2p_stream), ping_received_callback_(ping_received_callback) {}

  bool Run() override;
  bool OnRequest() override;

private:
  bool TrySendingReply();
  
  Status result_;
  enum { kProcessingRequest, kSendingReply } state_ = kProcessingRequest;
  PingReceivedCallbackType ping_received_callback_;
};

#endif  // PING_ACTION_HANDLER_
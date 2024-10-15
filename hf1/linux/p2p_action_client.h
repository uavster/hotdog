#ifndef P2P_ACTION_CLIENT_INCLUDED_
#define P2P_ACTION_CLIENT_INCLUDED_

#include "p2p_application_protocol.h"
#include "p2p_packet_stream_linux.h"
#include "timer_interface.h"

class P2PActionClient;

class P2PActionClientHandlerBase {
public:
  P2PActionClientHandlerBase(P2PAction action, P2PPacketStreamLinux *p2p_stream, bool allows_concurrent_requests = false)
    : action_(action), 
      p2p_stream_(*ASSERT_NOT_NULL(p2p_stream)), 
      allows_concurrent_requests_(allows_concurrent_requests), 
      request_length_(0),
      request_priority_(P2PPriority::kMedium),
      request_guarantee_delivery_(false),
      state_(kIdle) {}

  bool Request(int payload_length, const uint8_t *payload, P2PPriority priority, bool guarantee_delivery = false);
  bool Cancel(P2PPriority priority, bool guarantee_delivery = false);
  void Run();

  // Overrides must call the parent.
  virtual void OnReply(int payload_length, const uint8_t *payload) {
    std::lock_guard<std::mutex> guard(mutex_);
    state_ = kIdle;
  }
  virtual void OnProgress(int payload_length, const uint8_t *payload) {}
  
private:
  P2PAction action_;
  P2PPacketStreamLinux &p2p_stream_;
  bool allows_concurrent_requests_;

  std::mutex mutex_;

  int request_length_;
  uint8_t request_payload_[kP2PMaxContentLength];
  P2PPriority request_priority_;
  bool request_guarantee_delivery_;

  using State = enum { kIdle, kSendingRequest, kWaitingForReply, kSendingCancellation };
  State state_;

public:
  bool is_request_in_progress() const { 
    return !allows_concurrent_requests_ && request_length_ >= 0; }
};

template<typename TRequest, typename TReply, typename TProgress> class P2PActionClientHandler : public P2PActionClientHandlerBase {
public:
  // Does not take ownership of the pointees, which must outlive this object.
  P2PActionClientHandler(P2PAction action, P2PPacketStreamLinux *p2p_stream, bool allows_concurrent_requests = false)
    : P2PActionClientHandlerBase(action, p2p_stream, allows_concurrent_requests) {}
  
  bool Request(const TRequest &request, P2PPriority priority, bool guarantee_delivery = false) {
    return P2PActionClientHandlerBase::Request(sizeof(TRequest), &request, priority, guarantee_delivery);
  }

  virtual void OnReply(const TReply &reply) {}
  virtual void OnProgress(const TProgress &progress) {}

protected:
  void OnReply(int payload_length, const uint8_t *payload) override {    
    ASSERT(payload_length == sizeof(TReply));
    P2PActionClientHandlerBase::OnReply(payload_length, payload);
    OnReply(*reinterpret_cast<const TReply *>payload);
  }
  void OnProgress(int payload_length, const uint8_t *payload) override {
    ASSERT(payload_length == sizeof(TProgress));
    P2PActionClientHandlerBase::OnProgress(payload_length, payload);
    OnProgress(*reinterpret_cast<const TProgress *>payload);
  }
};

class P2PActionClient {
public:
  // Does not take ownership of the pointees, which must outlive this object.
  P2PActionClient(P2PPacketStreamLinux *p2p_stream, const TimerInterface *system_timer);

  // Does not take ownsership of the pointee, which must outlive this object.
  void Register(P2PActionClientHandlerBase *handler);

  // Dispatches reply and progress packets to handler callbacks.
  void Run();

  P2PPacketStreamLinux &p2p_stream() { return p2p_stream_; }

private:
  P2PPacketStreamLinux &p2p_stream_;
  const TimerInterface &system_timer_;
  P2PActionClientHandlerBase *handlers_[P2PAction::kCount];
};

#endif  // P2P_ACTION_CLIENT_INCLUDED_
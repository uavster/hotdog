#ifndef P2P_ACTION_CLIENT_INCLUDED_
#define P2P_ACTION_CLIENT_INCLUDED_

#include "p2p_application_protocol.h"
#include "p2p_packet_stream_linux.h"
#include "timer_interface.h"
#include <mutex>
#include <optional>

class P2PActionClient;

class P2PActionClientHandlerBase {
public:
  P2PActionClientHandlerBase(P2PAction action, P2PPriority priority, bool guarantee_delivery, P2PPacketStreamLinux *p2p_stream, std::mutex *p2p_mutex, bool allows_concurrent_requests = false)
    : action_(action), 
      priority_(priority),
      guarantee_delivery_(guarantee_delivery),
      p2p_stream_(*ASSERT_NOT_NULL(p2p_stream)), 
      current_request_id_(0),
      p2p_mutex_(*ASSERT_NOT_NULL(p2p_mutex)),
      allows_concurrent_requests_(allows_concurrent_requests), 
      state_(kIdle) {}

  bool Request(int payload_length, const void *payload, std::optional<P2PPriority> priority = std::nullopt, std::optional<bool> guarantee_delivery = std::nullopt);
  bool Cancel(std::optional<P2PPriority> priority = std::nullopt, std::optional<bool> guarantee_delivery = std::nullopt);

  // Overrides must call the parent.
  virtual void OnReply(int payload_length, const void *payload);
  virtual void OnProgress(int payload_length, const void *payload);

  P2PAction action() const { return action_; }
  // True if the action is being executed; false, otherwise.
  bool in_progress() const;
  P2PActionRequestID current_request_id() const { return current_request_id_; }
  
private:
  P2PAction action_;
  P2PPriority priority_;
  bool guarantee_delivery_;
  P2PPacketStreamLinux &p2p_stream_;
  P2PActionRequestID current_request_id_;
  std::mutex &p2p_mutex_;
  bool allows_concurrent_requests_;

  using State = enum { kIdle, kWaitingForResponse };
  State state_;
};

template<typename TRequest, typename TReply, typename TProgress> class P2PActionClientHandler : public P2PActionClientHandlerBase {
public:
  // Does not take ownership of the pointees, which must outlive this object.
  P2PActionClientHandler(P2PAction action, P2PPriority priority, bool guarantee_delivery, P2PPacketStreamLinux *p2p_stream, std::mutex *p2p_mutex, bool allows_concurrent_requests = false)
    : P2PActionClientHandlerBase(action, priority, guarantee_delivery, p2p_stream, p2p_mutex, allows_concurrent_requests) {}
  
  bool Request(const TRequest &request, std::optional<P2PPriority> priority = std::nullopt, std::optional<bool> guarantee_delivery = std::nullopt) {
    return P2PActionClientHandlerBase::Request(sizeof(TRequest), &request, priority, guarantee_delivery);
  }

  virtual void OnReply(const TReply &reply) {}
  virtual void OnProgress(const TProgress &progress) {}

protected:
  void OnReply(int payload_length, const void *payload) override {    
    ASSERT(payload_length == sizeof(TReply));
    P2PActionClientHandlerBase::OnReply(payload_length, payload);
    OnReply(*reinterpret_cast<const TReply *>(payload));
  }
  void OnProgress(int payload_length, const void *payload) override {
    ASSERT(payload_length == sizeof(TProgress));
    P2PActionClientHandlerBase::OnProgress(payload_length, payload);
    OnProgress(*reinterpret_cast<const TProgress *>(payload));
  }
};

class P2PActionClient {
public:
  // Does not take ownership of the pointees, which must outlive this object.
  P2PActionClient(P2PPacketStreamLinux *p2p_stream, const TimerInterface *system_timer);

  // Does not take ownsership of the pointee, which must outlive this object.
  void Register(P2PActionClientHandlerBase *handler);

  // Dispatches reply and progress packets to handler callbacks.
  // Should be called with the p2p_mutex passed to the P2PActionClientHandlers locked.
  void Run();

  P2PPacketStreamLinux &p2p_stream() { return p2p_stream_; }

private:
  P2PPacketStreamLinux &p2p_stream_;
  const TimerInterface &system_timer_;
  P2PActionClientHandlerBase *handlers_[P2PAction::kCount];
};

#endif  // P2P_ACTION_CLIENT_INCLUDED_
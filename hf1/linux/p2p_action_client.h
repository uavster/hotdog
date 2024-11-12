#ifndef P2P_ACTION_CLIENT_INCLUDED_
#define P2P_ACTION_CLIENT_INCLUDED_

#include "p2p_application_protocol.h"
#include "p2p_packet_stream_linux.h"
#include "timer_interface.h"
#include "logger_interface.h"
#include <mutex>
#include <optional>
#include <atomic>

class P2PActionClientHandlerBase {
public:  
  // The client has protected access to the action handlers.
  friend class P2PActionClient;

  P2PActionClientHandlerBase(P2PAction action, P2PPriority default_priority, bool default_guarantee_delivery, P2PPacketStreamLinux *p2p_stream, std::mutex *p2p_mutex, bool allows_concurrent_requests = false)
    : action_(action), 
      priority_(default_priority),
      guarantee_delivery_(default_guarantee_delivery),
      p2p_stream_(*ASSERT_NOT_NULL(p2p_stream)), 
      current_request_id_(0),
      p2p_mutex_(*ASSERT_NOT_NULL(p2p_mutex)),
      allows_concurrent_requests_(allows_concurrent_requests), 
      state_(kIdle) {}

  // Sends an action request message with the given `payload`.
  // If `priority` and `guarantee_delivery` are passed, they override the default
  // configuration passed in the constructor.
  // If successful, it resturn Status::kSuccess.
  // If the action is already in progress, it returns Status::kExistsError.
  // If no P2P packet slots are available to send the message, it returns Status::kUnavailableError.
  Status Request(int payload_length, const void *payload, std::optional<P2PPriority> priority = std::nullopt, std::optional<bool> guarantee_delivery = std::nullopt);

  // Sends an action cancellation message.
  // If `priority` and `guarantee_delivery` are passed, they override the default
  // configuration passed in the constructor.
  // If successful, it resturn Status::kSuccess.
  // If the action was not in progress, it returns Status::kDoesNotExistsError.
  // If no P2P packet slots are available to send the message, it returns Status::kUnavailableError.
  Status Cancel(std::optional<P2PPriority> priority = std::nullopt, std::optional<bool> guarantee_delivery = std::nullopt);

  P2PAction action() const { return action_; }
  // True if the action is being executed; false, otherwise.
  bool in_progress() const;

  // Overrides must call the parent.
  virtual void OnReply(int payload_length, const void *payload);
  virtual void OnProgress(int payload_length, const void *payload);

protected:
  // Must be called with p2p_mutex_ locked.
  P2PActionRequestID current_request_id() const { return current_request_id_; }

private:
  P2PAction action_;
  P2PPriority priority_;
  bool guarantee_delivery_;
  P2PPacketStreamLinux &p2p_stream_;
  P2PActionRequestID current_request_id_;
  std::mutex &p2p_mutex_;
  const bool allows_concurrent_requests_;

  using State = enum { kIdle, kWaitingForResponse };
  // Since the state is atomic, we can read it without locking.
  std::atomic<State> state_;
};

template<typename TRequest, typename TReply = P2PVoid, typename TProgress = P2PVoid> class P2PActionClientHandler : public P2PActionClientHandlerBase {
public:
  // Does not take ownership of the pointees, which must outlive this object.
  P2PActionClientHandler(P2PAction action, P2PPriority default_priority, bool default_guarantee_delivery, P2PPacketStreamLinux *p2p_stream, std::mutex *p2p_mutex, bool allows_concurrent_requests = false)
    : P2PActionClientHandlerBase(action, default_priority, default_guarantee_delivery, p2p_stream, p2p_mutex, allows_concurrent_requests) {}

  using OnReplyCallback = std::function<void(const TRequest &, const TReply &)>;
  using OnProgressCallback = std::function<void(const TRequest &, const TProgress &)>;

  // Takes ownsership of the callbacks.
  // See the base class' function for more details.
  Status Request(const TRequest &request, OnReplyCallback &&reply_callback, OnProgressCallback &&progress_callback, std::optional<P2PPriority> priority = std::nullopt, std::optional<bool> guarantee_delivery = std::nullopt) {
    last_request_ = request;
    reply_callback_ = reply_callback;
    progress_callback_ = progress_callback;
    return P2PActionClientHandlerBase::Request(sizeof(TRequest), &request, priority, guarantee_delivery);
  }

protected:
  virtual void OnReply(int payload_length, const void *payload) override {    
    ASSERT(payload_length == sizeof(TReply));
    P2PActionClientHandlerBase::OnReply(payload_length, payload);
    reply_callback_(last_request_, *reinterpret_cast<const TReply *>(payload));
  }
  virtual void OnProgress(int payload_length, const void *payload) override {
    ASSERT(payload_length == sizeof(TProgress));
    P2PActionClientHandlerBase::OnProgress(payload_length, payload);
    progress_callback_(last_request_, *reinterpret_cast<const TProgress *>(payload));
  }

private:
  TRequest last_request_;   // Action requests cannot overlap.
  OnReplyCallback reply_callback_;
  OnProgressCallback progress_callback_;
};

class P2PActionClient {
public:
  // Does not take ownership of the pointees, which must outlive this object.
  P2PActionClient(P2PPacketStreamLinux *p2p_stream, const TimerInterface *system_timer);

  // Does not take ownsership of the pointee, which must outlive this object.
  void Register(P2PActionClientHandlerBase *handler);
  P2PActionClientHandlerBase *HandlerForAction(P2PAction action) const { 
    ASSERT(action < P2PAction::kCount);
    return handlers_[action];
  }

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
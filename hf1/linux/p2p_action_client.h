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

  // Does not take ownership of the pointees, which must outlive this object.
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
  // It can return the following error codes:
  //  Status::kSuccess - the request was placed in the P2P output queue.
  //  Status::kUnavailableError - no P2P packet slots are available to send the message.
  //  Status::kExistsError - the action was already in progress.
  //  Status::kOverflowError - the P2P-encoded request packet does not fit in a slot of the output queue. 
  //    Encoded packets may be longer due to double-byte encoding of special characters.
  Status Request(int payload_length, const void *payload, std::optional<P2PPriority> priority = std::nullopt, std::optional<bool> guarantee_delivery = std::nullopt);

  // Sends an action cancellation message.
  // If `priority` and `guarantee_delivery` are passed, they override the default
  // configuration passed in the constructor.
  // It can return the following error codes:
  //  Status::kSuccess - the cancellation packet was placed in the P2P output queue.
  //  Status::kUnavailableError - no P2P packet slots are available to send the message.
  //  Status::kExistsError - the action was not in progress.
  Status Cancel(std::optional<P2PPriority> priority = std::nullopt, std::optional<bool> guarantee_delivery = std::nullopt);

  P2PAction action() const { return action_; }
  // True if the action is being executed; false, otherwise.
  bool in_progress() const;

protected:
  // Called when a reply is received.
  // Overrides must call the parent. Called with p2p_mutex locked.
  virtual void OnReply(int payload_length, const void *payload);

  // Called when a progress packet is received.
  // Overrides must call the parent. Called with p2p_mutex locked.
  virtual void OnProgress(int payload_length, const void *payload);

  // Called right after the other end starts.
  // Overrides must call the parent. Called with p2p_mutex locked.
  virtual void OnOtherEndStarted();

  // Called when the action could not be completed.
  // Takes the cancellation packet from the server as `payload`.
  // If the server was restarted, `payload` is nulptr.
  // Overrides must call the parent. Called with p2p_mutex locked.
  virtual void OnAbort(int payload_length, const void *payload);

  using State = enum { kIdle, kWaitingForResponse, kCancelling };

  // Must be called with p2p_mutex_ locked.
  P2PActionRequestID current_request_id() const { return current_request_id_; }
  State state() const { return state_; }
  // Manages the lifecycle of the action. 
  // Must be called from the action client after locking p2p_mutex_.
  void Run();

private:
  P2PAction action_;
  P2PPriority priority_;
  bool guarantee_delivery_;
  P2PPacketStreamLinux &p2p_stream_;
  P2PActionRequestID current_request_id_;
  std::mutex &p2p_mutex_;
  const bool allows_concurrent_requests_;

  // Since the state is atomic, we can read it without locking for single-read decisions, like in_progress().
  // For writing or multiple-read decisions, lock p2p_mutex_ first.
  std::atomic<State> state_;
};

template<typename TRequest> struct LiteralRequestSizeCalculator {
  static int CalculateRequestSize(const TRequest &r) { 
    return sizeof(TRequest); 
  }
};

template<typename TRequest> struct CreateTrajectoryRequestSizeCalculator {
  static int CalculateRequestSize(const TRequest &request) { 
    return sizeof(request) - sizeof(request.trajectory.waypoints) + request.trajectory.num_waypoints * sizeof(request.trajectory.waypoints[0]);
  }
};

template<typename TRequest, typename TReply = P2PVoid, typename TProgress = P2PVoid, typename TRequestSizeCalculator = LiteralRequestSizeCalculator<TRequest>> 
class P2PActionClientHandler : public P2PActionClientHandlerBase {
public:
  // Does not take ownership of the pointees, which must outlive this object.
  P2PActionClientHandler(P2PAction action, P2PPriority default_priority, bool default_guarantee_delivery, P2PPacketStreamLinux *p2p_stream, std::mutex *p2p_mutex, bool allows_concurrent_requests = false)
    : P2PActionClientHandlerBase(action, default_priority, default_guarantee_delivery, p2p_stream, p2p_mutex, allows_concurrent_requests) {}

  using OnReplyCallback = std::function<void(const TRequest &, const TReply &)>;
  using OnProgressCallback = std::function<void(const TRequest &, const TProgress &)>;
  using OnAbortCallback = std::function<void(const TRequest &, const StatusOr<TReply> &)>;

  // Takes ownsership of the callbacks.
  // See the base class' function for more details.
  Status Request(const TRequest &request, OnReplyCallback reply_callback, OnProgressCallback progress_callback, OnAbortCallback abort_callback, std::optional<P2PPriority> priority = std::nullopt, std::optional<bool> guarantee_delivery = std::nullopt) {
    last_request_ = request;
    reply_callback_ = std::move(reply_callback);
    progress_callback_ = std::move(progress_callback);
    abort_callback_ = std::move(abort_callback);
    return P2PActionClientHandlerBase::Request(TRequestSizeCalculator::CalculateRequestSize(request), &request, priority, guarantee_delivery);
    return Status::kSuccess;
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

  virtual void OnAbort(int payload_length, const void *payload) override {
    P2PActionClientHandlerBase::OnAbort(payload_length, payload);
    if (payload == nullptr) {
      // No payload means that aborting was triggered in the client (e.g. because the server was restarted).
      abort_callback_(last_request_, Status::kRestartedError);
      return;
    }
    ASSERT(payload_length == sizeof(TReply));
    abort_callback_(last_request_, *reinterpret_cast<const TReply *>(payload));
  }

private:
  TRequest last_request_;   // Action requests cannot overlap.
  OnReplyCallback reply_callback_;
  OnProgressCallback progress_callback_;
  OnAbortCallback abort_callback_;
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
  // Called when the other end is restarted.
  // Notifies all action handlers.
  static void OnOtherEndStarted(void *p_self);

  P2PPacketStreamLinux &p2p_stream_;
  const TimerInterface &system_timer_;
  P2PActionClientHandlerBase *handlers_[P2PAction::kCount];
};

#endif  // P2P_ACTION_CLIENT_INCLUDED_
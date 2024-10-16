#ifndef P2P_ACTION_SERVER_INCLUDED_
#define P2P_ACTION_SERVER_INCLUDED_

#include "p2p_packet_stream_arduino.h"
#include "p2p_application_protocol.h"
#include "utils.h"

class P2PActionHandlerBase;

// Provides typed access to an action packet payload.
template<typename TPacket> class P2PActionPacketAdapter {
public:
  // Empty constructor neede for StatusOr.
  P2PActionPacketAdapter() : action_handler_(nullptr) {}
  // Does not take ownership of the pointee, which must outlive this object.
  P2PActionPacketAdapter(P2PActionHandlerBase *action_handler, P2PMutablePacketView &packet_view)
    : action_handler_(ASSERT_NOT_NULL(action_handler)), packet_view_(packet_view) {}

  TPacket *operator->();
  void Commit(bool guarantee_delivery = false);

protected:
  P2PActionHandlerBase *action_handler_;
  P2PMutablePacketView packet_view_;
};

class P2PActionServer;

class P2PActionHandlerBase {
public:
  typedef enum { kIdle, kRunning } RunState;

  P2PActionHandlerBase(P2PAction action, P2PPacketStreamArduino *p2p_stream)
    : is_registered_(false), is_initialized_(false), action_(action), request_priority_(P2PPriority::kMedium), p2p_stream_(p2p_stream), run_state_(RunState::kIdle) {}

  bool is_registered() const { return is_registered_; }
  void is_registered(bool ir) { is_registered_ = ir; }
  bool is_initialized() const { return is_initialized_; }
  void is_initialized(bool ii) { is_initialized_ = ii; }  
  P2PAction action() const { return action_; }
  RunState run_state() const { return run_state_; }
  void run_state(RunState state) { run_state_ = state; }
  void request_priority(P2PPriority priority) { request_priority_ = priority; }
  P2PPriority request_priority() const { return request_priority_; }
  P2PActionRequestID request_id() const { return request_id_; }
  void request_id(P2PActionRequestID rid) { request_id_ = rid; }
  P2PPacketStreamArduino &p2p_stream() { return *p2p_stream_; }

  const uint8_t *request_bytes() const { return request_bytes_; }
  void request_bytes(const uint8_t *request_bytes) { request_bytes_ = request_bytes; }

  // Returns a pointer to a buffer where to copy the request when the action takes longer than a call to Run().
  virtual uint8_t *GetRequestCopyBuffer() = 0;

  // Returns the expected request size.
  virtual int GetExpectedRequestSize() const = 0;

  // Called once from the server's Run() before any other callbacks.
  virtual void Init() {}

  // Called when the action request is received. Returns whether the action will be started.
  // If false, the other end receives a P2PActionStage::kCancel stage; otherwise, the other 
  // end keeps waiting for P2PActionStage::kReply or P2PActionStage::kProgress.
  // Returns true if not implemented in the subclass.
  // When this is called, the request packet is guaranteed to be the oldest one in the input 
  // stream, if the handlers needs to access it.
  virtual bool OnRequest() { return true; }

  // Called continuously by the server's Run() if OnRequest() returned true, and while this 
  // returns true as well. The subclass is responsible for sending a reply or progress
  // updates.
  // The request packet is guaranteed to be the oldest one in the input stream only the first
  // time this is called. 
  virtual bool Run() = 0;

  // Called when the action requestor decides to cancel the action. If so, the handler 
  // should stop doing any work and, if needed, leave the robot in known state prior to
  // stopping.
  virtual void OnCancel() {}

private:
  bool is_registered_;
  bool is_initialized_;
  P2PAction action_;
  P2PPriority request_priority_;
  P2PActionRequestID request_id_;
  P2PPacketStreamArduino *p2p_stream_;
  RunState run_state_;
  P2PPacketView app_packet_view_;
  const uint8_t *request_bytes_;
};

typedef struct {} VoidPacket;

template<typename TRequest, typename TReply = VoidPacket, typename TProgress = VoidPacket> class P2PActionHandler : public P2PActionHandlerBase {
public:
  P2PActionHandler(P2PAction action, P2PPacketStreamArduino *p2p_stream)
    : P2PActionHandlerBase(action, p2p_stream) {}

  // Returns the request that triggered the action.
  // The returned reference is guaranteed to be valid throughout the handler's lifetime,
  // but it should not be cached across callbacks as it might change over time.
  const TRequest &GetRequest() const;

  // Creates a TReply in a new output stream packet or returns an error status.
  StatusOr<P2PActionPacketAdapter<TReply>> NewReply();

  // Creates a TProgress in a new output packet or returns an error status.
  StatusOr<P2PActionPacketAdapter<TProgress>> NewProgress();

  int GetExpectedRequestSize() const override {
    return sizeof(TRequest);
  }

  uint8_t *GetRequestCopyBuffer() override {
    return reinterpret_cast<uint8_t *>(&request_);
  }

private:
  TRequest request_;
};

class P2PActionServer {
public:
  // Does not take ownership of the pointee, which must outlive this object.
  P2PActionServer(P2PPacketStreamArduino *p2p_stream);
  virtual ~P2PActionServer();

  // Registers an action handler.
  // Does not take ownership of the pointee, which must outlive this object.
  void Register(P2PActionHandlerBase *handler);

  // Runs the server. Must be called in a run loop.
  void Run();

private:
  void InitActionsIfNeeded();
  void RunActions();
  StatusOr<const P2PPacketView> GetRequestOrCancellation() const;

  P2PPacketStreamArduino &p2p_stream_;
  P2PActionHandlerBase *handlers_[P2PAction::kCount];
};

#include "p2p_action_server.hh"

#endif  // P2P_ACTION_SERVER_INCLUDED_
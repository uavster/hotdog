#ifndef P2P_APPLICATION_PROTOCOL_
#define P2P_APPLICATION_PROTOCOL_

#include <stdint.h>

// Action identifiers go in the 6 upper bits of the command field. The 2 lower bits indicate
// the action's stage: whether it is a request, a reply, a cancellation or a progress report.
// Upon this, we can implement messages, services or actions (RPCs).

// The type of action to execute.
// There can be a maximum of 64 different actions, as the action is represented with 6 bits.
// Actions can only be started from one end. Please create a new action, if an action should
// be started from the machine at the other end.
// Reordering the existing commands will require both ends to be rebuilt to understand each
// other. Adding new commands AFTER the existing ones will still produce 
// backward-compatible binaries; obviously, the side that is not updated will not understand
// the new commands.
typedef enum {
  kPing = 0,
  kTimeSync,
  kSetHeadPose,
  kSetBaseVelocity,
  kMonitorBaseState,

  kCount  // Must be the last entry in the enum.
} P2PAction;

// The stage at which the action is at.
// There can be a maximum of 4 possible stages, as the stage is represented with 2 bits.
typedef enum {
  kRequest = 0,
  kReply,
  kCancel,
  kProgress
} P2PActionStage;

#pragma pack(push, 1)

typedef uint8_t P2PActionRequestID;

typedef struct {
    uint8_t action : 6;   // The type of action [P2PAction].
    uint8_t stage: 2;     // The action stage [P2PActionStage].
    // Identifies the action request. 
    // This is a monotonically increasing counter for action requests. Action cancellations, 
    // replies and progress updates must indicate in this field the ID of the action request
    // they refer to. 
    // This field can only disambiguate as many requests as different values it can hold; 
    // this limits the number of request-cancellation pairs that can coexist in the output 
    // packet stream.
    P2PActionRequestID request_id;  
} P2PApplicationPacketHeader;

// Void action.
typedef struct {} P2PVoid;

// Time synchronization.
typedef struct {
    uint64_t sync_edge_local_timestamp_ns;
} P2PSyncTimeRequest;

typedef struct {
    uint64_t sync_edge_local_timestamp_ns;
} P2PSyncTimeReply;

// Set head pose.
typedef struct {
    float pitch_radians;
    float roll_radians;
} P2PSetHeadPoseRequest;

// Set base velocity.
typedef struct {
    float forward_meters_per_second;
    float counterclockwise_radians_per_second;
} P2PSetBaseVelocityRequest;

// Monitor estimated base state.
typedef struct {
    // Maximum number of messages that can be sent back, including progress and
    // result. Once reached, the server sends a reply and the action ends. If 0, 
    // progress updates are sent non-stop and the client is responsible for 
    // cancelling the action when done.
    uint16_t max_updates;
} P2PMonitorBaseStateRequest;

typedef struct {
    uint64_t global_timestamp_ns;
    // Position of the middle point of the axis passing through wheel centers
    // with respect to the robot's local frame when it was powered up.
    // The x vector points to the then robot's front. The y vector points to the
    // then robot's left.
    float position_x;
    float position_y;
    // Rotation angle around the z vector, which points to the sky, following
    // the right hand rule (counterclockwise, looking at the robot from above).
    float yaw;
} P2PMonitorBaseStateProgress;

typedef P2PMonitorBaseStateProgress P2PMonitorBaseStateReply;

#pragma pack(pop)

#endif  // P2P_APPLICATION_PROTOCOL_
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

typedef struct {
    uint8_t action : 6;
    uint8_t stage: 2;
} P2PApplicationPacketHeader;

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

#pragma pack(pop)

#endif  // P2P_APPLICATION_PROTOCOL_
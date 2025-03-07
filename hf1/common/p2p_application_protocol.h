#ifndef P2P_APPLICATION_PROTOCOL_
#define P2P_APPLICATION_PROTOCOL_

#include <stdint.h>

// A trajectory must fit in a P2P packet. Set the maximum number of waypoints taking
// this into account and the size of the largest waypoint type.
#define kP2PMaxNumWaypointsPerBaseTrajectory 10
#define kP2PMaxNumWaypointsPerHeadTrajectory 20
#define kP2PMaxNumWaypointsPerEnvelopeTrajectory 10

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
  kCreateBaseTrajectory,
  kCreateHeadTrajectory,
  kCreateEnvelopeTrajectory,
  kCreateBaseTrajectoryView,
  kCreateHeadTrajectoryView,
  kCreateEnvelopeTrajectoryView,
  kCreateBaseModulatedTrajectoryView,
  kCreateHeadModulatedTrajectoryView,
  kCreateBaseMixedTrajectoryView,
  kCreateHeadMixedTrajectoryView,
  kExecuteBaseTrajectoryView,
  kExecuteHeadTrajectoryView,

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

// --- Void action ---
typedef struct {} P2PVoid;

// --- Time synchronization ---
typedef struct {
    uint64_t sync_edge_local_timestamp_ns;
} P2PSyncTimeRequest;

typedef struct {
    uint64_t sync_edge_local_timestamp_ns;
} P2PSyncTimeReply;

// --- Set head pose ---
typedef struct {
    float pitch_radians;
    float roll_radians;
} P2PSetHeadPoseRequest;

// --- Set base velocity ---
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

// --- Create base trajectory ---
typedef struct {
  float x_meters;
  float y_meters;
  float yaw_radians;
} P2PBaseStateVars;

typedef struct {
  P2PBaseStateVars location;
} P2PBaseTargetState;

typedef struct {
  float seconds;
  P2PBaseTargetState target_state;
} P2PBaseWaypoint;

typedef struct {
  uint32_t num_waypoints;
  P2PBaseWaypoint waypoints[kP2PMaxNumWaypointsPerBaseTrajectory];
} P2PBaseTrajectory;

typedef struct {
  uint8_t id;
  P2PBaseTrajectory trajectory;
} P2PCreateBaseTrajectoryRequest;

typedef struct {
  uint8_t status_code;
} P2PCreateBaseTrajectoryReply;

// --- Create head trajectory ---
typedef struct {
  float pitch_radians;
  float roll_radians;
} P2PHeadStateVars;

typedef struct {
  P2PHeadStateVars location;
} P2PHeadTargetState;

typedef struct {
  float seconds;
  P2PHeadTargetState target_state;
} P2PHeadWaypoint;

typedef struct {
  int num_waypoints;
  P2PHeadWaypoint waypoints[kP2PMaxNumWaypointsPerHeadTrajectory];
} P2PHeadTrajectory;

typedef struct {
  int id;
  P2PHeadTrajectory trajectory;
} P2PCreateHeadTrajectoryRequest;

typedef struct {
  uint8_t status_code; // A Status code.
} P2PCreateHeadTrajectoryReply;

// --- Create envelope trajectory ---
typedef struct {
  float value;
} P2PEnvelopeStateVars;

typedef struct {
  P2PEnvelopeStateVars location;
} P2PEnvelopeTargetState;

typedef struct {
  float seconds;
  P2PEnvelopeTargetState target_state;
} P2PEnvelopeWaypoint;

typedef struct {
  int num_waypoints;
  P2PEnvelopeWaypoint waypoints[kP2PMaxNumWaypointsPerEnvelopeTrajectory];
} P2PEnvelopeTrajectory;

typedef struct {
  int id;
  P2PEnvelopeTrajectory trajectory;
} P2PCreateEnvelopeTrajectoryRequest;

typedef struct {
  uint8_t status_code; // A Status code.
} P2PCreateEnvelopeTrajectoryReply;

// --- Create base trajectory view ---
typedef enum {
  kNone = 0,
  kLinear = 1,
  kCubic = 2  // Centripetal Catmull-Rom splines.
} P2PInterpolationType;

typedef struct {
  uint8_t type; // P2PInterpolationType
} P2PTrajectoryInterpolationConfig;

typedef struct {
  // Identifier of the trajectory on which this view operates.
  uint8_t trajectory_id;

  // If >= 0, the trajectory loops this number of seconds.
  // If < 0, the trajectory does not loop.
  float loop_after_seconds;

  P2PTrajectoryInterpolationConfig interpolation_config;
} P2PBaseTrajectoryView;

typedef struct {
  uint8_t id;
  P2PBaseTrajectoryView trajectory_view;
} P2PCreateBaseTrajectoryViewRequest;

typedef struct {
  uint8_t status_code;
} P2PCreateBaseTrajectoryViewReply;

// --- Create head trajectory view ---
typedef struct {
  // Identifier of the trajectory on which this view operates.
  uint8_t trajectory_id;

  // If >= 0, the trajectory loops this number of seconds.
  // If < 0, the trajectory does not loop.
  float loop_after_seconds;

  P2PTrajectoryInterpolationConfig interpolation_config;
} P2PHeadTrajectoryView;

typedef struct {
  uint8_t id;
  P2PHeadTrajectoryView trajectory_view;
} P2PCreateHeadTrajectoryViewRequest;

typedef struct {
  uint8_t status_code;
} P2PCreateHeadTrajectoryViewReply;

// --- Create envelope trajectory view ---
typedef struct {
  // Identifier of the trajectory on which this view operates.
  uint8_t trajectory_id;

  // If >= 0, the trajectory loops this number of seconds.
  // If < 0, the trajectory does not loop.
  float loop_after_seconds;

  P2PTrajectoryInterpolationConfig interpolation_config;
} P2PEnvelopeTrajectoryView;

typedef struct {
  uint8_t id;
  P2PEnvelopeTrajectoryView trajectory_view;
} P2PCreateEnvelopeTrajectoryViewRequest;

typedef struct {
  uint8_t status_code;
} P2PCreateEnvelopeTrajectoryViewReply;

// --- Create base modulated trajectory view ---
typedef enum {
  kPlain = 0,
  kModulated,
  kMixed,

  kNumTrajectoryViewTypes
} P2PTrajectoryViewType;

typedef struct {
  uint8_t type; // P2PTrajectoryViewType.
  uint8_t id;
} P2PTrajectoryViewID;

typedef struct {
  // Identifier of the base trajectory view that takes the base from 
  // point A to point B.
  P2PTrajectoryViewID carrier_trajectory_view_id;

  // Identifier of the base trajectory view that modulates the carrier.
  P2PTrajectoryViewID modulator_trajectory_view_id;

  // Identifier of the envelope trajectory view that multiplies the 
  // modulated carrier.
  uint8_t envelope_trajectory_view_id;
} P2PModulatedTrajectoryView;

typedef struct {
  uint8_t id;
  P2PModulatedTrajectoryView trajectory_view;
} P2PCreateBaseModulatedTrajectoryViewRequest;

typedef struct {
  uint8_t status_code;
} P2PCreateBaseModulatedTrajectoryViewReply;

// --- Create head modulated trajectory view ---
typedef struct {
  uint8_t id;
  P2PModulatedTrajectoryView trajectory_view;
} P2PCreateHeadModulatedTrajectoryViewRequest;

typedef struct {
  uint8_t status_code;
} P2PCreateHeadModulatedTrajectoryViewReply;

// --- Create base mixed trajectory view ---
typedef struct {
  // ID of the first trajectory view to mix. 
  // It can be a view to any trajectory whose state type is equal to that of 
  // the second view, e.g. both are base trajectories, both are head trajectories.
  P2PTrajectoryViewID first_trajectory_view_id;

  // ID of the second trajectory view to mix.
  // It can be a view to any trajectory whose state type is equal to that of 
  // the first view, e.g. both are base trajectories, both are head trajectories.
  P2PTrajectoryViewID second_trajectory_view_id;

  // ID of the envelope trajectoy view controlling the mix factor between 
  // the first and second trajectories. When the envelope is 0, this view
  // is equivalent to the first view. When it is 1, it is equivalient to the
  // second view.
  uint8_t alpha_envelope_trajectory_view_id;
} P2PMixedTrajectoryView;

typedef struct {
  uint8_t id;
  P2PMixedTrajectoryView trajectory_view;
} P2PCreateBaseMixedTrajectoryViewRequest;

typedef struct {
  uint8_t status_code;
} P2PCreateBaseMixedTrajectoryViewReply;

// --- Create head modulated trajectory view ---
typedef struct {
  uint8_t id;
  P2PMixedTrajectoryView trajectory_view;
} P2PCreateHeadMixedTrajectoryViewRequest;

typedef struct {
  uint8_t status_code;
} P2PCreateHeadMixedTrajectoryViewReply;

// --- Execute base trajectory view ---
typedef struct {
  P2PTrajectoryViewID trajectory_view_id;
} P2PExecuteBaseTrajectoryViewRequest;

typedef struct {
  float num_completed_laps;
} P2PExecuteBaseTrajectoryViewProgress;

typedef struct {
  uint8_t status_code;
} P2PExecuteBaseTrajectoryViewReply;

// --- Execute head trajectory view ---
typedef struct {
  P2PTrajectoryViewID trajectory_view_id;
} P2PExecuteHeadTrajectoryViewRequest;

typedef struct {
  float num_completed_laps;
} P2PExecuteHeadTrajectoryViewProgress;

typedef struct {
  uint8_t status_code;
} P2PExecuteHeadTrajectoryViewReply;

#pragma pack(pop)

const char *GetTrajectoryViewTypeName(P2PTrajectoryViewType type);

#endif  // P2P_APPLICATION_PROTOCOL_
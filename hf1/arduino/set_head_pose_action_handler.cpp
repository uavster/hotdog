#include "set_head_pose_action_handler.h"
#include "servos.h"
#include "operation_mode.h"

bool SetHeadPoseActionHandler::Run() {
  const P2PSetHeadPoseRequest &request = GetRequest();
  const float yaw_radians = NetworkToLocal<kP2PLocalEndianness>(request.yaw_radians);
  const float pitch_radians = NetworkToLocal<kP2PLocalEndianness>(request.pitch_radians);
  const float roll_radians = NetworkToLocal<kP2PLocalEndianness>(request.roll_radians);

  char str[64];
  sprintf(str, "set_head_pose(yaw=%f, pitch=%f, roll=%f)\n", yaw_radians, pitch_radians, roll_radians);
  LOG_INFO(str);

  EnableTrajectoryControl(false);
  
  SetHeadYawDegrees(DegreesFromRadians(yaw_radians));
  SetHeadRollDegrees(DegreesFromRadians(roll_radians));
  SetHeadPitchDegrees(DegreesFromRadians(pitch_radians));

  return false;   // The action is complete: do not call Run() again.
}
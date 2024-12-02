#include "set_head_pose_action_handler.h"
#include "servos.h"

bool SetHeadPoseActionHandler::Run() {
  const P2PSetHeadPoseRequest &request = GetRequest();
  const float pitch_radians = NetworkToLocal<kP2PLocalEndianness>(request.pitch_radians);
  const float roll_radians = NetworkToLocal<kP2PLocalEndianness>(request.roll_radians);

  char str[64];
  sprintf(str, "set_head_pose(pitch=%f, roll=%f)\n", pitch_radians, roll_radians);
  LOG_INFO(str);
  
  SetHeadRollDegrees((roll_radians * 180.0f) / M_PI);
  SetHeadPitchDegrees((pitch_radians * 180.0f) / M_PI);

  return false;   // The action is complete: do not call Run() again.
}
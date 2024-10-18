#include "set_base_velocity_action_handler.h"
#include "wheel_controller.h"

bool SetBaseVelocityActionHandler::Run() {
  const P2PSetHeadPoseRequest &request = GetRequest();
  const float linear_speed = NetworkToLocal<kP2PLocalEndianness>(request.forward_meters_per_second);
  const float angular_speed = NetworkToLocal<kP2PLocalEndianness>(request.counterclockwise_radians_per_second);

  Serial.printf("set_base_velocity(linear=%f, angular=%f)\n", linear_speed, angular_speed);
  
  wheel_speed_controller_.SetLinearSpeed(linear_speed);
  wheel_speed_controller_.SetAngularSpeed(angular_speed);

  return false;   // The action is complete: do not call Run() again.
}
#include "set_base_velocity_action_handler.h"
#include <algorithm>
#include "motors.h"
#include "operation_mode.h"

bool SetBaseVelocityActionHandler::Run() {
  const P2PSetBaseVelocityRequest &request = GetRequest();
  const float linear_speed = NetworkToLocal<kP2PLocalEndianness>(request.forward_meters_per_second);
  const float angular_speed = NetworkToLocal<kP2PLocalEndianness>(request.counterclockwise_radians_per_second);

  // Serial.printf("set_base_velocity(linear=%f, angular=%f)\n", linear_speed, angular_speed);
  
  // base_speed_controller_.SetTargetSpeeds(linear_speed, angular_speed);

  const float kAngularFactor = 0.4;
  const float kLinearFactor = 2.0;

  float left_angular = kLinearFactor * linear_speed - kAngularFactor * angular_speed;
  float left_duty = left_angular;
  left_duty = std::clamp(left_duty, -1.0f, 1.0f);

  float right_angular = kLinearFactor * linear_speed + kAngularFactor * angular_speed;
  float right_duty = right_angular;
  right_duty = std::clamp(right_duty, -1.0f, 1.0f);

  EnableWheelControl(false);
  SetLeftMotorDutyCycle(left_duty);
  SetRightMotorDutyCycle(right_duty);
  // base_speed_controller_.left_wheel_speed_controller().SetAngularSpeed(linear_speed - angular_speed);
  // base_speed_controller_.right_wheel_speed_controller().SetAngularSpeed(linear_speed + angular_speed);
  // SetLeftMotorDutyCycle(float s)

  return false;   // The action is complete: do not call Run() again.
}
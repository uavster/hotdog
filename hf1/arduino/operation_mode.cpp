static bool is_trajectory_control_enabled;
static bool is_wheel_control_enabled;

bool EnableTrajectoryControl(bool e) {
  bool old_value = is_trajectory_control_enabled;
  is_trajectory_control_enabled = e;  
  return old_value;
}

bool EnableWheelControl(bool e) {
  bool old_value = is_wheel_control_enabled;
  is_wheel_control_enabled = e;
  return old_value;
}

bool IsTrajectoryControlEnabled() {
  return is_trajectory_control_enabled;
}

bool IsWheelControlEnabled() {
  return is_wheel_control_enabled;
}


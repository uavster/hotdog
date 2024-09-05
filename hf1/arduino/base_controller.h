#ifndef ROBOT_SPEED_CONTROLLER_
#define ROBOT_SPEED_CONTROLLER_

#include "wheel_controller.h"
#include "periodic_runnable.h"
#include "robot_state.h"
#include "timer.h"
#include "status_or.h"

// Controller commanding the wheel speed controllers to achieve the desired forward and 
// angular speeds of the robot's base.
class BaseSpeedController {
public:
  // Does not take ownership of any pointee.
  BaseSpeedController(WheelSpeedController *left_wheel, WheelSpeedController *right_wheel);

  // Sets the target linear and angular speeds. The speeds might not be attainable by the 
  // wheels. If the linear speed is not attainable, the target is clamped to the wheel limit. 
  // Then, if the angular speed is not attainable, it is clamped to the angular speed limit 
  // given the updated target linear speed, and the linear speed is adjusted to keep the 
  // radius given by the updated linear speed and the original angular speed.
  void SetTargetSpeeds(float linear, float angular);

  // Returns the target linear speed adjusted to what the wheels can do.
  float target_linear_speed() const { return target_speed_linear_; }
  // Returns the target angular speed adjusted to what the wheels can do.
  float target_angular_speed() const { return target_speed_angular_; }
  // Returns the radius of the curve given the adjusted target speeds.
  float curve_radius() const { return target_linear_speed() / target_angular_speed(); }

  void Run();

  const WheelSpeedController &left_wheel_speed_controller() const { return left_wheel_; }
  const WheelSpeedController &right_wheel_speed_controller() const { return right_wheel_; }

private:
  WheelSpeedController &left_wheel_;
  WheelSpeedController &right_wheel_;
  float target_speed_linear_;
  float target_speed_angular_;
};

// Controller commanding the base speed controller to achieve the desired position and yaw
// of the robot's base within a tolerance. Convergence is not guaranteed in any given time
// horizon. 
class BaseStateController : public PeriodicRunnable {
public:
  BaseStateController(BaseSpeedController *base_speed_controller);

  void SetTargetState(const Point &center_position_target, float yaw_target, float reference_forward_speed, float reference_angular_speed = 0.0f);

  void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override;
  void Run();

  const BaseSpeedController &base_speed_controller() const { return base_speed_controller_; }

  bool IsAtTargetState() const;

private:
  BaseSpeedController &base_speed_controller_;
  Point center_position_target_;
  float yaw_target_;
  float reference_forward_speed_;
  float reference_angular_speed_;
};

// Defines the state of the robot's base at a given time. The controller class decides what
// part of the state to use. For instance, some controllers may ignore the time and/or the 
// yaw angle.
class BaseWaypoint {
public:
  BaseWaypoint() : seconds_(0), yaw_(0) {}
  BaseWaypoint(TimerSecondsType seconds, const Point &position, float yaw) 
    : seconds_(seconds), position_(position), yaw_(yaw) {}

  TimerSecondsType seconds() const { return seconds_; }
  const Point &position() const { return position_; }
  float yaw() const { return yaw_; }

private:
  TimerSecondsType seconds_;
  Point position_;
  float yaw_;
};

// A view of a collection of base waypoints.
// The view does not own the memory containing the waypoints, so they must outlive any 
// view objects referencing them.
class BaseTrajectoryView {
public:
  BaseTrajectoryView() : num_waypoints_(0), waypoints_(NULL) {}

  // Does not take ownsership of the pointee, which must outlive this object.
  BaseTrajectoryView(int num_waypoints, const BaseWaypoint *waypoints)
    : num_waypoints_(num_waypoints), waypoints_(waypoints) {}

  int num_waypoints() const { return num_waypoints_; }

  StatusOr<int> FindWaypointIndexBeforeSeconds(TimerSecondsType seconds, int prev_result_index = 0) const;

  const Point &position(int index) const;
  Point velocity(int index) const;
  Point acceleration(int index) const;
  float seconds(int index) const;

private:
  int num_waypoints_;
  const BaseWaypoint *waypoints_;
};

// Controller commanding the base speed controller to move the robot's base over a sequence
// of waypoints. It tries to reach each waypoint at the waypoint's time. The base will not
// drive over a waypoint if it was not able to reach it on time. A far away waypoint with 
// a time very near to the previous waypoint's time will not be reachable. Also, any obstacle 
// and driving hurdle or error can result in not reaching a waypoint in time. When the 
// waypoint's time contraint cannot be met and the waypoint is the last one in the 
// trajectory, the robot will stop. But if the waypoint is not the last one, the robot will 
// skip to the next one.
// 
// This controller is based on the paper:
// R. L. S. Sousa, M. D. do Nascimento Forte, F. G. Nogueira, B. C. Torrico, 
// “Trajectory tracking control of a nonholonomic mobile robot with differential drive”, 
// in Proc. IEEE Biennial Congress of Argentina (ARGENCON), pp. 1–6, 2016.
class BaseTrajectoryController : public PeriodicRunnable {
public:
  BaseTrajectoryController(BaseSpeedController *base_speed_controller);

  void trajectory(const BaseTrajectoryView &trajectory);

  void StartTrajectory();
  void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override;
  void Run();

private:
  BaseSpeedController &base_speed_controller_;
  BaseTrajectoryView trajectory_;
  int current_waypoint_index_;
  bool is_started_;
  TimerSecondsType start_seconds_;
};

#endif  // ROBOT_SPEED_CONTROLLER_
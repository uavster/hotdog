#ifndef ROBOT_STATE_INCLUDED_
#define ROBOT_STATE_INCLUDED_

#include <Kalman.h>
#include "robot_model.h"
#include "timer.h"

// Somewhere down the third-party header tree, a macro F is defined, which makes it impossible to access kalman_.F.
#undef F

class Point {
  public:
    float x;
    float y;

    Point(): x(0), y(0) {}
    Point(float x_, float y_) : x(x_), y(y_) {}
    Point operator-(const Point &p) { return Point(x - p.x, y - p.y); }
    float norm() const { return sqrtf(x * x + y * y); }
};

#define kNumStateVars 5
#define kNumObservationVars 5
#define kNumCommandVars 3

class BaseState {
  public:
    BaseState();
    
    void NotifyWheelTicks(TimerTicksType timer_ticks, int left_ticks_inc, int right_ticks_inc);
    void NotifyLeftWheelDirection(bool backward);
    void NotifyRightWheelDirection(bool backward);

    void NotifyIMUReading(TimerTicksType timer_ticks, float accel_x, float accel_y, float yaw);
    void EstimateState(TimerTicksType timer_ticks);

    Point center() const;
    Point center_velocity() const;
    float yaw() const;

  private:
    // Odometry.
    TimerTicksType last_odom_timer_ticks_;
    int left_wheel_ticks_;
    int right_wheel_ticks_;
    bool left_wheel_moving_backward_;
    bool right_wheel_moving_backward_;
    Point odom_center_;
    Point odom_center_velocity_;
    float odom_yaw_;

    // IMU.
    TimerTicksType last_imu_timer_ticks_;
    Point imu_acceleration_;
    float imu_yaw_;

    TimerTicksType last_state_update_timer_ticks_;    
    // Requires BasicLinearAlgebra 3.7 or lower in order to compile (see https://github.com/rfetick/Kalman/issues/9#issuecomment-2225218225).
    KALMAN<kNumStateVars, kNumObservationVars, kNumCommandVars, /*MemF=*/TriangularSup<kNumStateVars, float>> kalman_;
};

#endif  // ROBOT_STATE_INCLUDED_
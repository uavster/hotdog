#include "robot_model.h"

class Point {
  public:
    float x;
    float y;

    Point(): x(0), y(0) {}
};

class RobotState {
  private:
    int left_wheel_ticks_;
    int right_wheel_ticks_;
    float angle_;
    Point center_;
    bool left_wheel_moving_backward_;
    bool right_wheel_moving_backward_;

  public:
    RobotState();
    
    void NotifyWheelTicks(int left_ticks_inc, int right_ticks_inc);

    void NotifyLeftWheelDirection(bool backward);

    void NotifyRightWheelDirection(bool backward);

    const Point &Center();

    const float Angle();
};
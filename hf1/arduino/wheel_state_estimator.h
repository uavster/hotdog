#ifndef WHEEL_STATE_ESTIMATOR_INCLUDED_
#define WHEEL_STATE_ESTIMATOR_INCLUDED_

#include "timer.h"
#include "status_or.h"
#include "periodic_runnable.h"
#include "robot_model.h"

class WheelState {
  friend class WheelStateFilter;
public:
  WheelState() : linear_speed_(0) {}
  WheelState(float speed) : linear_speed_(speed) {}
  float angular_speed() const { return linear_speed_ / kWheelRadius; }
  float linear_speed() const { return linear_speed_; }

protected:
  void linear_speed(float speed) { linear_speed_ = speed; };
private:
  float linear_speed_;
};

class WheelStateFilter {
  friend class WheelStateEstimator;
public:
  WheelStateFilter() 
    : last_encoder_edge_timer_ticks_(Status::kDoesNotExistError) {}

  WheelState state() const;

  StatusOr<TimerTicksType> last_encoder_edge_timer_ticks() const { return last_encoder_edge_timer_ticks_; }

protected:
  void UpdateState();
  void NotifyEncoderEdge(TimerTicksType timer_ticks);

private:
  StatusOr<TimerTicksType> last_encoder_edge_timer_ticks_;
  WheelState wheel_state_;
};

class WheelStateEstimator : public PeriodicRunnable {
public:
  WheelStateEstimator(const char *name);

  static void Init();

  const WheelStateFilter &left_wheel_state_filter() const { return left_wheel_state_filter_; }
  const WheelStateFilter &right_wheel_state_filter() const { return right_wheel_state_filter_; }

protected:
  static void LeftEncoderIsr(TimerTicksType timer_ticks);
  static void RightEncoderIsr(TimerTicksType timer_ticks);
  void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override;

private:
  WheelStateFilter left_wheel_state_filter_;
  WheelStateFilter right_wheel_state_filter_;
};

#endif  // WHEEL_STATE_ESTIMATOR_INCLUDED_

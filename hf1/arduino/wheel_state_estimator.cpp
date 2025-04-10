#include "wheel_state_estimator.h"
#include "encoders.h"
#include "timer.h"
#include "utils.h"
#include "robot_model.h"
#include "logger_interface.h"

// Approximate rate at which the state estimation is updated.
#define kUpdatePeriodSeconds (1/160.0)

// Target fraction of the last measured wheel speed fed to the filter after kWheelSpeedDecaySeconds.
#define kWheelSpeedDecayReduction 1e-2
// Seconds required to reduce the wheel speed to kOdomCenterVelocityDecayReduction of the last measurement.
#define kWheelSpeedDecaySeconds 1.0
#define kWheelSpeedDecayFactor (exp((log(kWheelSpeedDecayReduction) * kUpdatePeriodSeconds) / kWheelSpeedDecaySeconds))

static WheelStateEstimator *wheel_state_estimator_singleton = nullptr;

void WheelStateEstimator::LeftEncoderIsr(TimerTicksType timer_ticks) {  
  ASSERT(wheel_state_estimator_singleton != nullptr);
  wheel_state_estimator_singleton->left_wheel_state_filter_.NotifyEncoderEdge(timer_ticks);
}

void WheelStateEstimator::RightEncoderIsr(TimerTicksType timer_ticks) {
  ASSERT(wheel_state_estimator_singleton != nullptr);
  wheel_state_estimator_singleton->right_wheel_state_filter_.NotifyEncoderEdge(timer_ticks);
}

WheelStateEstimator::WheelStateEstimator(const char *name) 
  : PeriodicRunnable(name, static_cast<TimerSecondsType>(kUpdatePeriodSeconds)) {
  ASSERT(wheel_state_estimator_singleton == nullptr);
  wheel_state_estimator_singleton = this;
}

void WheelStateEstimator::Init() {
  AddEncoderIsrs(&WheelStateEstimator::LeftEncoderIsr, &WheelStateEstimator::RightEncoderIsr);  
}

void WheelStateEstimator::RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) {
  left_wheel_state_filter_.UpdateState();
  right_wheel_state_filter_.UpdateState();  
}

void WheelStateFilter::NotifyEncoderEdge(TimerTicksType timer_ticks) {
  if (last_encoder_edge_timer_ticks_.ok()) {
    const auto timer_ticks_between_encoder_edges = timer_ticks - *last_encoder_edge_timer_ticks_;
    wheel_state_.linear_speed((kWheelRadius * kRadiansPerWheelTick) / SecondsFromTimerTicks(timer_ticks_between_encoder_edges));
  }
  last_encoder_edge_timer_ticks_ = timer_ticks;
}

void WheelStateFilter::UpdateState() {
  NO_ENCODER_IRQ {
    // When no encoder edges arrive, assume the wheel slows down to prevent estimating 
    // constant speed when it's actually stopped.
    wheel_state_.linear_speed(wheel_state_.linear_speed() * kWheelSpeedDecayFactor);  
  }
}

WheelState WheelStateFilter::state() const {
  NO_ENCODER_IRQ {
    return wheel_state_;
  }
  return WheelState();  // Avoid compiler warning.
}
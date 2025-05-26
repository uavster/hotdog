#include <BasicLinearAlgebra.h>
#include "base_state_filter.h"


// Standard deviations of the IMU measures.
#define kStdevIMUAccelX 0.0118  // [m/s^2] Estimated from accelerometer data in https://forums.adafruit.com/viewtopic.php?t=122078
#define kStdevIMUAccelY 0.0128  // [m/s^2] Estimated from accelerometer data in https://forums.adafruit.com/viewtopic.php?t=122078
#define kStdevIMUYaw 0.001   // [rad] Estimated experimentally. The measure comes from the IMU internal filter, so noise is pretty low.

// Standard deviations of the odometry measures.
#define kOdomSamplingRate 160.0
#define kStdevOdomPosX (0.005 / (0.85 * kOdomSamplingRate))  // +/-5 mm error in a 0.85-seconds test (21.5 cm/s) at the approximate sample rate [m]
#define kStdevOdomPosY (0.005 / (0.85 * kOdomSamplingRate))  // +/-5 mm error in a 0.85-seconds test (21.5 cm/s) at the approximate sample rate [m]
// For a 90-degree turn in 1 second, we get +/-10 degrees of error. However, testing different
// trajectories, we see the error can be much higher for sudden movements, so we apply an error
// multiplier to account for the worst case, as it can have devastating effects on the
// localization accuracy in the long term. In this way, yaw is mostly determined by the IMU
// measurement whose accuracy is stable across our acceleration range.
#define kStdevOdomYaw (100 * (M_PI / 180 * (10 / (90.0 / 90.0 * kOdomSamplingRate))))  // +/-10 degreed after 90 degrees turn, at 90 deg/s and the approximate sample rate [rad]

// Any estimate above this value is rejected.
// Meant to prevent too high estimates due to co-occuring encoder edges.
#define kOdomCenterSpeedMax 2.0 // [m/s]

// The odometry model applies a factor in [0, 1] to the velocity estimate at every update 
// to make it decay. Otherwise, even after the robot stops, velocity stays at the estimate
// from the last encoder edge. The factor is calculated so that either dimension of the 
// velocity is its last value multiplied by kOdomCenterVelocityDecayReduction after
// kOdomCenterVelocityDecaySeconds seconds.

// Target fraction of the last measured odometry velocity fed to the filter after kOdomCenterVelocityDecaySeconds.
#define kOdomCenterVelocityDecayReduction 1e-3
// Seconds required to reduce the odometry center velocity to kOdomCenterVelocityDecayReduction of the last measurement.
#define kOdomCenterVelocityDecaySeconds ((kWheelRadius * kRadiansPerWheelTick) / 0.5)  // The time between wheel ticks at 0.5 m/s.
// Time constant of the decay factor applied to the last odometry center measure: exp(-time_since_last_odom_center_measure / kOdomCenterDecayTimeConstant)
#define kOdomCenterDecayTimeConstant (-kOdomCenterVelocityDecaySeconds / log(kOdomCenterVelocityDecayReduction))

BaseStateFilter::BaseStateFilter() 
  : last_odom_timer_ticks_(0), 
    left_wheel_ticks_(0), 
    right_wheel_ticks_(0), 
    left_wheel_moving_backward_(false), 
    right_wheel_moving_backward_(false), 
    odom_yaw_(0.0f), 
    last_imu_timer_ticks_(0), 
    imu_yaw_(0.0f), 
    last_yaw_estimate_(0), 
    last_state_update_timer_ticks_(0) {     
  
  // F: state transition model. x_k = F*x_{k-1} + B*u_k + w_k.
  const float time_inc = 1e-3f;  // This value really doesn't matter. It's just makes the matrices below easier to understand.
  // This is a first-order model of the robot's position. The robot's yaw comes entirely
  // from the IMU ("command") and the odometry, so its coefficient is zero in the state
  // transition model.
  kalman_.F = { 1, 0, time_inc, 0, 0, 0, 
                0, 1, 0, time_inc, 0, 0, 
                0, 0, 1, 0, 0, 0, 
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0
              };

  // B: control-input model. x_k = F*x_{k-1} + B*u_k + w_k.
  const float time_inc_2 = time_inc * time_inc;
  kalman_.B = {
                0.5f * time_inc_2, 0, 0, 0,
                0, 0.5f * time_inc_2, 0, 0,
                time_inc, 0, 0, 0, 
                0, time_inc, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1
              };

  // H: observation model. y_k = H*x_k + v_k.
  kalman_.H = { 1, 0, 0, 0, 0, 0, 
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1
              };

  // Q: covariance matrix of process noise.
  const float time_inc_4 = time_inc_2 * time_inc_2;
  const float qpx = kStdevIMUAccelX * kStdevIMUAccelX * time_inc_4;
  const float qpy = kStdevIMUAccelY * kStdevIMUAccelY * time_inc_4;
  const float qvx = kStdevIMUAccelX * kStdevIMUAccelX * time_inc_2;
  const float qvy = kStdevIMUAccelY * kStdevIMUAccelY * time_inc_2;
  const float qpa = kStdevIMUYaw * kStdevIMUYaw;
  // Since yaw is represented as a complex number, we need the noise variance in each component.
  // Assume that the noise mean is 0.
  const float one_minus_exp_minus_qpa = 1 - expf(-qpa);
  const float qcospa = 0.5f * one_minus_exp_minus_qpa * one_minus_exp_minus_qpa;
  const float qsinpa = 0.5f * (1.0f - expf(-2.0f * qpa));
  kalman_.Q = {
                qpx, 0, 0, 0, 0, 0,
                0, qpy, 0, 0, 0, 0,
                0, 0, qvx, 0, 0, 0,
                0, 0, 0, qvy, 0, 0,
                0, 0, 0, 0, qcospa, 0,
                0, 0, 0, 0, 0, qsinpa
              };

  // R: covariance matrix of observation noise.
  const float rpx = kStdevOdomPosX * kStdevOdomPosX;  // [m^2]
  const float rpy = kStdevOdomPosY * kStdevOdomPosY;  // [m^2]
  const float rpa = kStdevOdomYaw * kStdevOdomYaw;  // [rad^2]
  // Velocity is calculated from position, 
  const float rvx = (2 * rpx * rpx) / time_inc_2;  // [(m/s)^2]
  const float rvy = (2 * rpy * rpy) / time_inc_2;  // [(m/s)^2]
  // Since yaw is represented as a complex number, we need the noise variance in each component.
  // Assume that the noise mean is 0.
  const float one_minus_exp_minus_rpa = 1 - expf(-rpa);
  const float rcospa = 0.5f * one_minus_exp_minus_rpa * one_minus_exp_minus_rpa;
  const float rsinpa = 0.5f * (1.0f - expf(-2.0f * rpa));
  kalman_.R = {
                rpx, 0, 0, 0, 0, 0,
                0, rpy, 0, 0, 0, 0,
                0, 0, rvx, 0, 0, 0,
                0, 0, 0, rvy, 0, 0,
                0, 0, 0, 0, rcospa, 0,
                0, 0, 0, 0, 0, rsinpa
              };
}

void BaseStateFilter::EstimateState(TimerTicksType timer_ticks) {
  // Recalculate F with Ts=time_since_last_filter_run, and re-run Kalman filter.
  const float state_update_timer_inc = SecondsFromTimerTicks(timer_ticks - last_state_update_timer_ticks_);
  kalman_.F(0, 2) = state_update_timer_inc;
  kalman_.F(1, 3) = state_update_timer_inc;
  
  // Update state estimation.
  // Avoid yaw discontinuities messing with the Kalman estimate by representing the yaw as a
  // complex number on the unit circle.
  // Decay odometry velocity, so it goes to zero if no more wheel ticks are received.  
  const float odom_velocity_decay_factor = expf(-SecondsFromTimerTicks(timer_ticks - last_odom_timer_ticks_) / kOdomCenterDecayTimeConstant);
  kalman_.update(/*obs=*/{ odom_center_.x, odom_center_.y, odom_center_velocity_.x * odom_velocity_decay_factor, odom_center_velocity_.y * odom_velocity_decay_factor, cosf(odom_yaw_), sinf(odom_yaw_) }, 
                 /*command=*/{ imu_acceleration_.x, imu_acceleration_.y, cosf(imu_yaw_), sinf(imu_yaw_) });

  // Serial.printf("F = {{ %f %f %f %f %f }, {%f %f %f %f %f}, {%f %f %f %f %f}, { %f %f %f %f %f }, {%f %f %f %f %f}}\n", kalman_.F(0, 0), kalman_.F(0, 1), kalman_.F(0, 2), kalman_.F(0, 3), kalman_.F(0, 4), kalman_.F(1, 0), kalman_.F(1, 1), kalman_.F(1, 2), kalman_.F(1, 3), kalman_.F(1, 4), kalman_.F(2, 0), kalman_.F(2, 1), kalman_.F(2, 2), kalman_.F(2, 3), kalman_.F(2, 4), kalman_.F(3, 0), kalman_.F(3, 1), kalman_.F(3, 2), kalman_.F(3, 3), kalman_.F(3, 4), kalman_.F(4, 0), kalman_.F(4, 1), kalman_.F(4, 2), kalman_.F(4, 3), kalman_.F(4, 4));
  // Serial.printf("B = {{ %f %f %f }, {%f %f %f}, {%f %f %f}, {%f %f %f }, {%f %f %f}}\n", kalman_.B(0, 0), kalman_.B(0, 1), kalman_.B(0, 2), kalman_.B(1, 0), kalman_.B(1, 1), kalman_.B(1, 2), kalman_.B(2, 0), kalman_.B(2, 1), kalman_.B(2, 2), kalman_.B(3, 0), kalman_.B(3, 1), kalman_.B(3, 2), kalman_.B(4, 0), kalman_.B(4, 1), kalman_.B(4, 2));
  // Serial.printf("H = {{ %f %f %f %f %f}, {%f %f %f %f %f}, {%f %f %f %f %f}, {%f %f %f %f %f}, {%f %f %f %f %f}}\n", kalman_.H(0, 0), kalman_.H(0, 1), kalman_.H(0, 2), kalman_.H(0, 3), kalman_.H(0, 4), kalman_.H(1, 0), kalman_.H(1, 1), kalman_.H(1, 2), kalman_.H(1, 3), kalman_.H(1, 4), kalman_.H(2, 0), kalman_.H(2, 1), kalman_.H(2, 2), kalman_.H(2, 3), kalman_.H(2, 4), kalman_.H(3, 0), kalman_.H(3, 1), kalman_.H(3, 2), kalman_.H(3, 3), kalman_.H(3, 4), kalman_.H(4, 0), kalman_.H(4, 1), kalman_.H(4, 2), kalman_.H(4, 3), kalman_.H(4, 4));  
  
  // Yaw velocity is not key, so we estimate it roughly outside the Kalman filter to keep
  // the matrices smaller.
  if (state_update_timer_inc > 0) {
    const float current_yaw_estimate = atan2f(kalman_.x(5), kalman_.x(4));
    yaw_velocity_ = (current_yaw_estimate - last_yaw_estimate_) / state_update_timer_inc;
    last_yaw_estimate_ = current_yaw_estimate;
  }

  last_state_update_timer_ticks_ = timer_ticks;
}

void BaseStateFilter::NotifyWheelTicks(TimerTicksType timer_ticks, int left_ticks_inc, int right_ticks_inc) {
  if (left_wheel_moving_backward_) {
    left_ticks_inc = -left_ticks_inc;
  }
  if (right_wheel_moving_backward_) {
    right_ticks_inc = -right_ticks_inc;
  }
  left_wheel_ticks_ += left_ticks_inc;
  right_wheel_ticks_ += right_ticks_inc;
  float distance_inc;
  float distance_inc_yaw;
  if (left_ticks_inc != right_ticks_inc) {
    const float odom_yaw_inc = ((kRadiansPerWheelTick * kWheelRadius) * (right_ticks_inc - left_ticks_inc)) / kRobotDistanceBetweenTireCenters;
    const float perimeter_inc = (kRadiansPerWheelTick * kWheelRadius * (left_ticks_inc + right_ticks_inc)) / 2;
    const float curve_radius = perimeter_inc / abs(odom_yaw_inc);
    distance_inc = curve_radius * sqrtf(2.0f * (1.0f - cosf(odom_yaw_inc)));
    distance_inc_yaw = GetFilteredYaw() + 0.5f * odom_yaw_inc;
  } else {
    distance_inc = (kRadiansPerWheelTick * kWheelRadius * (left_ticks_inc + right_ticks_inc)) / 2;
    distance_inc_yaw = GetFilteredYaw();
  }
  const float x_inc = distance_inc * cosf(distance_inc_yaw);
  const float y_inc = distance_inc * sinf(distance_inc_yaw);
  odom_yaw_ = ((kRadiansPerWheelTick * kWheelRadius) * (right_wheel_ticks_ - left_wheel_ticks_)) / kRobotDistanceBetweenTireCenters;

  const float odom_timer_inc = SecondsFromTimerTicks(timer_ticks - last_odom_timer_ticks_);
  if (odom_timer_inc >= sqrtf(x_inc * x_inc + y_inc * y_inc) / kOdomCenterSpeedMax) {
    // There could be input capture edges in the buffer before the timer started
    odom_center_velocity_.x = x_inc / odom_timer_inc;
    odom_center_velocity_.y = y_inc / odom_timer_inc;
  }
  last_odom_timer_ticks_ = timer_ticks;
  odom_center_.x += x_inc;
  odom_center_.y += y_inc;
  // Serial.printf("left_wheel_ticks_:%d right_wheel_ticks_:%d\n", left_wheel_ticks_, right_wheel_ticks_);
  // Serial.printf("odom_yaw_:%f distance_inc:%f distance_inc_yaw:%f x_inc:%f y_inc:%f odom_center_x:%f odom_center_y:%f odom_center_vx:%f odom_center_vy:%f\n", odom_yaw_, distance_inc, distance_inc_yaw, x_inc, y_inc, odom_center_.x, odom_center_.y, odom_center_velocity_.x, odom_center_velocity_.y);

  // Update the observation covariance elements that depend on the odometry sampling period.
  // This is an approximation under the assumption that time_inc is constant. It might not be
  // accurate looking at the entire signal because the sampling rate depends on wheel speed, 
  // but it is meant to downplay velocity estimates coming from position samples very close 
  // in time, where errors will be more amplified.
  const float odom_time_inc_2 = odom_timer_inc * odom_timer_inc;
  kalman_.R(2, 2) = (2 * kalman_.R(0, 0) * kalman_.R(0, 0)) / odom_time_inc_2;
  kalman_.R(3, 3) = (2 * kalman_.R(1, 1) * kalman_.R(1, 1)) / odom_time_inc_2;

  EstimateState(timer_ticks);
}

void BaseStateFilter::NotifyLeftWheelDirection(bool backward) {
  left_wheel_moving_backward_ = backward;
}

void BaseStateFilter::NotifyRightWheelDirection(bool backward) {
  right_wheel_moving_backward_ = backward;
}

void BaseStateFilter::NotifyIMUReading(TimerTicksType timer_ticks, float accel_x, float accel_y, float yaw) {
  const float imu_time_inc = SecondsFromTimerTicks(timer_ticks - last_imu_timer_ticks_);
  last_imu_timer_ticks_ = timer_ticks;

  const float cos_yaw = cosf(yaw);
  const float sin_yaw = sinf(yaw);
  imu_acceleration_.x = accel_x * cos_yaw - accel_y * sin_yaw;
  imu_acceleration_.y = accel_x * sin_yaw + accel_y * cos_yaw;
  imu_yaw_ = yaw;

  // Serial.printf("imu_accel_.x:%f imu_accel_.y:%f imy_yaw_:%f\n", imu_acceleration_.x, imu_acceleration_.y, imu_yaw_);

  // Update the control-input elements that depend on the IMU sampling period.
  const float imu_time_inc_2 = imu_time_inc * imu_time_inc;
  kalman_.B(0, 0) = 0.5f * imu_time_inc_2;
  kalman_.B(1, 1) = kalman_.B(0, 0);
  kalman_.B(2, 0) = imu_time_inc;
  kalman_.B(3, 1) = imu_time_inc;

  // Update the process covariance elements that depend on the IMU sampling period.
  const float imu_time_inc_4 = imu_time_inc_2 * imu_time_inc_2;
  kalman_.Q(0, 0) = kStdevIMUAccelX * kStdevIMUAccelX * imu_time_inc_4;
  kalman_.Q(1, 1) = kStdevIMUAccelY * kStdevIMUAccelY * imu_time_inc_4;
  kalman_.Q(2, 2) = kStdevIMUAccelX * kStdevIMUAccelX * imu_time_inc_2;
  kalman_.Q(3, 3) = kStdevIMUAccelY * kStdevIMUAccelY * imu_time_inc_2;

  EstimateState(timer_ticks);
}

float BaseStateFilter::GetFilteredYaw() const {
  return atan2f(kalman_.x(5), kalman_.x(4));
}

BaseState BaseStateFilter::state() const  {
  // As dmuir's answer above points out, we have to normalize the estimated yaw state, too.
  // When the state is at the transition edge between pi and -pi, the innovation (eventhough
  // normalized) may take the state above pi or below -pi.
  return BaseState({ 
    BaseStateVars(Point(kalman_.x(0), kalman_.x(1)), GetFilteredYaw()), 
    BaseStateVars(Point(kalman_.x(2), kalman_.x(3)), yaw_velocity_)
  });
}

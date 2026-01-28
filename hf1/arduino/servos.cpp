// Control code for SG90 servos.

#include "kinetis.h"
#include "servos.h"
#include "Arduino.h"
#include <algorithm>
#include "persistance_offsets.h"
#include "persistance.h"

// Servo configuration.

typedef struct {
  float yaw;
  float pitch;
  float roll;
} ServoAngles;

typedef struct {
  ServoAngles command_offsets;
  ServoAngles feedback_offsets;
} ServoCalibrationData;

ServoCalibrationData servo_calibration_data;
ServoAngles command_degrees;

// Frequency of the servo command pulses. 
// The higher it is, the more time resolution in trajectories.
// SG90 servos documentation states nominal frequency is 50 Hz, but most (if not all) servos handle 100 Hz ok, 
// which gives us more time resolution.
constexpr float kServoPWMFrequencyHz = 100.0f;

constexpr float kServoMsPer180Degrees = 2.0f;
constexpr float kServoZeroDegreePulseMs = 1.5f;

// At 96MHz, the bus frequency is 48MHz, and we get 3000 clock ticks for the total 2ms range of pulse width, i.e. 180/3000 = 0.06 degrees resolution.
constexpr float kTimerTicksPerSecond = (F_BUS / 32); // System clock (bus clock for FTM) / 32 prescaler (PS = 0b100).
constexpr float kPWMPeriodTicks = (kTimerTicksPerSecond / kServoPWMFrequencyHz);

constexpr float kMinServoDegrees = -90.0f;
constexpr float kMaxServoDegrees = 90.0f;

constexpr float kMinYawDegrees = -90.0f;
constexpr float kMaxYawDegrees = 90.0f;

constexpr float kMinPitchDegrees = -60.0f;
constexpr float kMaxPitchDegrees = 60.0f;

constexpr float kMinRollDegrees = -45.0f;
constexpr float kMaxRollDegrees = 45.0f;

constexpr int32_t kMinus90DegreesFeedbackADCValue = static_cast<uint32_t>((1024 * 0.6f) / 3.3f);
constexpr int32_t kPlus90DegreesFeedbackADCValue = static_cast<uint32_t>((1024 * 2.45f) / 3.3f);

void LoadServoCalibration();

void InitServos() {
  FTM0_SC = 0;
  // Enable with QUADEN=0, DECAPEN=0, COMBINE=0, CPWMS=0, MSnB=1
  FTM0_QDCTRL = 0;
  FTM0_COMBINE = 0;
  FTM0_MOD = kPWMPeriodTicks - 1;
  FTM0_CNTIN = 0;
  FTM0_CNT = 0;
  FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0b101);  // CPWMS=0, CLKS=System clock, PS=0b111 (divide clock by 32).
  // Set edge-aligned PWM.
  // CH1IE = 0 (interrupt disabled), MS1B:MS0A = 2 and ELS1B:ELS1A = 2 (high-true pulses)
  FTM0_C2SC = FTM_CSC_MSB | FTM_CSC_ELSB; // pitch.
  FTM0_C3SC = FTM_CSC_MSB | FTM_CSC_ELSB; // roll.
  FTM0_C4SC = FTM_CSC_MSB | FTM_CSC_ELSB; // yaw.

  // Set FTM0_CHx function for pins.
  PORTC_PCR3 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; // pitch.
  PORTC_PCR4 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; // roll.
  PORTD_PCR4 = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; // yaw.

  // Load servo offsets.
  LoadServoCalibration();

  SetHeadYawDegrees(0);
  SetHeadPitchDegrees(0);
  SetHeadRollDegrees(0);
}

static float SaturateRoll(float angle_degrees) {
  return std::clamp(angle_degrees, kMinRollDegrees, kMaxRollDegrees);
}

static float SaturatePitch(float angle_degrees) {
  return std::clamp(angle_degrees, kMinPitchDegrees, kMaxPitchDegrees);
}

static float SaturateYaw(float angle_degrees) {
  return std::clamp(angle_degrees, kMinYawDegrees, kMaxYawDegrees);
}

static float SaturateServoMaxAngle(float angle_degrees) {
  return std::clamp(angle_degrees, kMinServoDegrees, kMaxServoDegrees);
}

void SetHeadPitchDegrees(float angle_degrees) {
  const float corrected_angle = SaturatePitch(angle_degrees) + servo_calibration_data.command_offsets.pitch;
  command_degrees.pitch = corrected_angle;
  float pulse_width_ms = (SaturateServoMaxAngle(-corrected_angle) * kServoMsPer180Degrees) / 180 + kServoZeroDegreePulseMs;
  FTM0_C2V = (pulse_width_ms * kTimerTicksPerSecond) / 1000;
}

void SetHeadRollDegrees(float angle_degrees) {
  const float corrected_angle = SaturateRoll(angle_degrees) + servo_calibration_data.command_offsets.roll;
  command_degrees.roll = corrected_angle;
  float pulse_width_ms = (SaturateServoMaxAngle(corrected_angle) * kServoMsPer180Degrees) / 180 + kServoZeroDegreePulseMs;
  FTM0_C3V = (pulse_width_ms * kTimerTicksPerSecond) / 1000;
}

void SetHeadYawDegrees(float angle_degrees) {
  const float corrected_angle = SaturateYaw(angle_degrees) + servo_calibration_data.command_offsets.yaw;
  command_degrees.yaw = corrected_angle;
  float pulse_width_ms = (SaturateServoMaxAngle(corrected_angle) * kServoMsPer180Degrees) / 180 + kServoZeroDegreePulseMs;
  FTM0_C4V = (pulse_width_ms * kTimerTicksPerSecond) / 1000;
}

float ServoOrientationDegreesFromFeedbackADCValue(int adc_value) {
  return 180.0f * (adc_value - kMinus90DegreesFeedbackADCValue) / (kPlus90DegreesFeedbackADCValue - kMinus90DegreesFeedbackADCValue) - 90.0;
}

float GetHeadYawRawDegrees() {
  return ServoOrientationDegreesFromFeedbackADCValue(analogRead(A1));
}

float GetHeadYawDegrees() {
  return GetHeadYawRawDegrees() + servo_calibration_data.feedback_offsets.yaw;
}

float GetHeadPitchRawDegrees() {
  return ServoOrientationDegreesFromFeedbackADCValue(analogRead(A8));
}

float GetHeadPitchDegrees() {
  return -(GetHeadPitchRawDegrees() + servo_calibration_data.feedback_offsets.pitch);
}

float GetHeadRollRawDegrees() {
  return ServoOrientationDegreesFromFeedbackADCValue(analogRead(A9));
}

float GetHeadRollDegrees() {
  return GetHeadRollRawDegrees() + servo_calibration_data.feedback_offsets.roll;
}

void LoadServoCalibration() {
  const auto data_size = persistent_storage.get<uint8_t>(kPersistanceOffsetServoCalibration);
  ASSERT(data_size.ok());
  if (*data_size != sizeof(ServoCalibrationData)) {
    LOG_WARNING("Unable to load servo calibration data. Please recalibrate!");
    servo_calibration_data.command_offsets = { 0, 0, 0 };
    return;
  }
  const auto calibration = persistent_storage.get<ServoCalibrationData>(kPersistanceOffsetServoCalibration + 1);
  ASSERT(calibration.ok());
  servo_calibration_data = *calibration;
}

void SaveServoAnglesAsOrigin() {
  servo_calibration_data.command_offsets = command_degrees;
  servo_calibration_data.feedback_offsets.yaw = -GetHeadYawRawDegrees();
  servo_calibration_data.feedback_offsets.pitch = -GetHeadPitchRawDegrees();
  servo_calibration_data.feedback_offsets.roll = -GetHeadRollRawDegrees();
  ASSERT(persistent_storage.put(kPersistanceOffsetServoCalibration, static_cast<uint8_t>(sizeof(ServoCalibrationData))) == Status::kSuccess);
  ASSERT(persistent_storage.put(kPersistanceOffsetServoCalibration + 1, servo_calibration_data) == Status::kSuccess);
}

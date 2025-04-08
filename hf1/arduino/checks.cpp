#include <limits>
#include "wiring.h"
#include "kinetis.h"
#include "checks.h"
#include "p2p_packet_stream_arduino.h"
#include "mcu_id.h"
#include "battery.h"
#include "motors.h"
#include "timer.h"
#include <util/atomic.h>
#include "encoders.h"
#include "robot_model.h"
#include "body_imu.h"
#include "operation_mode.h"
#include <EEPROM.h>

constexpr float kMinBatteryVoltage = 7.0f;
constexpr float kDefaultWaitInputTimeoutSeconds = 8.0f;

// Returns true if enter was pressed, or false if the timeout expired.
static bool WaitForEnterOrTimeout(Stream &stream, float timeout_seconds = kDefaultWaitInputTimeoutSeconds, bool print_default_string = true) {
  if (print_default_string) {
    stream.printf("[Press ENTER when done or wait for %.1f seconds]", timeout_seconds);
  }
  const auto start_seconds = GetTimerSeconds();
  while(!stream.available() && GetTimerSeconds() - start_seconds < timeout_seconds) {}
  if (stream.available()){
    while(stream.available()) {
      stream.read();
    }
    return true;
  }
  return false;
}

static void PrintIndentation(Stream &stream, int num_spaces) {
  while(num_spaces) { stream.print(" "); --num_spaces; }
}

bool CheckMCU(Stream &stream) {
  MCUID mcu_id;
  mcu_id.Print(stream);
  const bool success = mcu_id.family_id == 1 && mcu_id.pin_id == 0b0101;
  if (success) {
    stream.println("OK.");
  } else {
    stream.println("ERROR: The MCU should be a 64-pin K20.");
  }
  return success;
}

bool CheckSRAM(Stream &stream) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // Write to every DWORD in SRAM, check write was ok and restore the value.
    const int mem_size = 64 * 1024;
    stream.printf("Checking %d bytes of SRAM...\n", mem_size);
    volatile uint32_t *p = reinterpret_cast<volatile uint32_t *>(0x20000000 - mem_size / 2);
    for (size_t i = 0; i < mem_size / sizeof(uint32_t); ++i, ++p) {
      const uint32_t old_value = *p;
      const uint32_t increment = 0x12345678;
      *p += increment;
      uint32_t test_mask = (*p) ^ (old_value + increment);
      if (test_mask) {
        *p = old_value;
        for (size_t j = 0; j < sizeof(uint32_t) / sizeof(uint8_t); ++j) {
          if (test_mask & 0xff) {
#if kP2PLocalEndianness == kLittleEndian
            const volatile uint8_t *bad_address = reinterpret_cast<volatile uint8_t *>(p) + j;
#else
            const volatile uint8_t *bad_address = reinterpret_cast<volatile uint8_t *>(p) + (sizeof(uint32_t) / sizeof(uint8_t) - 1 - j);
#endif
            stream.printf("ERROR: SRAM is damaged at %p.\n", bad_address);
            break;
          }
          test_mask >>= 8;
        }
        return false;
      }
      *p = old_value;
    }

    // Check all the above for the remaining bytes.
    const int size_reminder = mem_size % sizeof(uint32_t);
    volatile uint8_t *bp = reinterpret_cast<volatile uint8_t *>(p);
    for (int i = 0; i < size_reminder; ++i, ++bp) {
      const uint8_t old_value = *bp;
      const uint8_t increment = 0x52;
      *bp += increment;
      if (*bp != old_value + increment) {
        *bp = old_value;
        stream.printf("ERROR: SRAM is damaged at %p.\n", bp);
        return false;
      }
      *bp = old_value;
    }

    stream.println("OK.");
  }
  return true;
}

bool CheckEEPROM(Stream &stream) {
  stream.printf("Checking %d bytes of EEPROM...\n", EEPROM.length());
  int first_error_index = -1;
  int num_errors = 0;
  for (int i = EEPROM.begin(); i < EEPROM.end(); ++i) {
    uint8_t value = EEPROM.read(i);
    const uint8_t test_value = value + 0x52;
    EEPROM.write(i, test_value);
    if (EEPROM.read(i) != test_value) {
      if (first_error_index < 0) {
        first_error_index = i;
      }
      ++num_errors;
    }
    EEPROM.write(i, value);
  }
  if (num_errors > 0) {
    stream.printf("ERROR: EEPROM has %d damaged positions, starting at 0x%x.\n", num_errors, first_error_index);
    return false;
  }

  stream.println("OK.");
  return true;
}

bool CheckBattery(Stream &stream) {
  const float voltage = GetBatteryVoltage();
  const bool ok = voltage >= kMinBatteryVoltage;
  stream.printf("Battery is at %.1fV. The minimum allowed level is %.1fV.\n", voltage, kMinBatteryVoltage);
  if (ok) {
    stream.print("OK.");
  } else {
    stream.print("ERROR: Battery voltage is too low.");
  }
  stream.println();
  return ok;
}

bool CheckTimer(Stream &stream) {
  volatile uint64_t accum = 0;  
  const uint64_t max_n = 5'000'000;
  stream.println("Timing an expensive calculation...");
  const float start_s = GetTimerSeconds();
  // The following code should take approximately 0.73 seconds on a K20 at 96 MHz.
  for (uint64_t i = 0; i < max_n; ++i) {
    accum += i;
  }
  const float elapsed_s = GetTimerSeconds() - start_s;
  if (accum != max_n * (max_n - 1) / 2) {
    stream.println("ERROR: Wrong test result. Please check the SRAM.");
    return false;
  }
  constexpr float kMinElapsedSeconds = 0.725f;
  constexpr float kMaxElapsedSeconds = 0.735f;
  const bool ok = elapsed_s >= kMinElapsedSeconds && elapsed_s <= kMaxElapsedSeconds;
  if (!ok) {
    stream.printf("ERROR: The test took %f s, but it was expected to take something in [%f, %f] s.\n", elapsed_s, kMinElapsedSeconds, kMaxElapsedSeconds);
    return false;
  }
  stream.println("OK.");
  return true;
}

bool CheckMotors(Stream &stream, bool check_preconditions) {
  if (check_preconditions) {
    stream.println("Running precondition tests.");
    if (!CheckTimer()) {
      stream.println("ERROR: The MCU timer is not working as expected.");
      return false;
    }
    if (!CheckBattery()) {
      stream.println("ERROR: Battery voltage is to low to run this test.");
      return false;
    }  
  }

  const bool old_wheel_control_status = EnableWheelControl(false);
  const bool old_trajectory_control_status = EnableTrajectoryControl(false);

  // Move each wheel backward and forward.
  constexpr float kAtomicMotionSeconds = 0.5f;
  stream.printf("Spinning left wheel backward for %.1f seconds...\n", kAtomicMotionSeconds);
  SetLeftMotorDutyCycle(-0.7);
  SleepForSeconds(kAtomicMotionSeconds);
  stream.printf("Spinning left wheel forward for %.1f seconds...\n", kAtomicMotionSeconds);
  SetLeftMotorDutyCycle(0.7);
  SleepForSeconds(kAtomicMotionSeconds);
  SetLeftMotorDutyCycle(0);

  stream.printf("Spinning right wheel backward for %.1f seconds...\n", kAtomicMotionSeconds);
  SetRightMotorDutyCycle(-0.7);
  SleepForSeconds(kAtomicMotionSeconds);
  stream.printf("Spinning right wheel forward for %.1f seconds...\n", kAtomicMotionSeconds);
  SetRightMotorDutyCycle(0.7);
  SleepForSeconds(kAtomicMotionSeconds);
  SetRightMotorDutyCycle(0);

  stream.println("If the wheels did not spin back and forth one at a time (first left, then right), check the hardware.");

  EnableWheelControl(old_wheel_control_status);
  EnableTrajectoryControl(old_trajectory_control_status);

  return true;
}

static volatile uint8_t num_left_ticks;
static volatile uint8_t num_right_ticks;

static void GotLeftEncoderTick(TimerTicksType timer_ticks) {
  ++num_left_ticks;
}

static void GotRightEncoderTick(TimerTicksType timer_ticks) {
  ++num_right_ticks;
}

bool CheckEncoders(Stream &stream, bool check_preconditions) {
  if (check_preconditions) {
    stream.println("Running precondition tests...");
    if (!CheckTimer()) {
      stream.println("ERROR: The MCU timer is not working as expected.");
      return false;
    }
  }

  num_left_ticks = 0;
  num_right_ticks = 0;
  AddEncoderIsrs(&GotLeftEncoderTick, &GotRightEncoderTick);

  // Wait for the user to spin each wheel and count the encoder ticks in the meantime.
  constexpr float kTicksPerWheelTurn = 2 * M_PI / kRadiansPerWheelTick;
  constexpr float kMaxErrorNumTurns = 0.2;
  constexpr int kMaxErrorTicks = static_cast<int>(kMaxErrorNumTurns * kTicksPerWheelTurn);
  constexpr int kMinTicks = kTicksPerWheelTurn - kMaxErrorTicks;
  constexpr int kMaxTicks = kTicksPerWheelTurn + kMaxErrorTicks;
  constexpr int kResultIndentationNumSpaces = 3;

  // Left wheel.
  bool left_ok = false;
  stream.printf("1. Spin the LEFT wheel ONE FULL TURN ONLY in either direction. ");
  WaitForEnterOrTimeout(stream); stream.println();
  if (num_left_ticks >= kMinTicks && num_left_ticks <= kMaxTicks) {
    PrintIndentation(stream, kResultIndentationNumSpaces);
    stream.printf("OK: Left wheel spinned %.2f times.\n", num_left_ticks / kTicksPerWheelTurn);
    left_ok = true;
  } else {
    if (num_left_ticks == 0) {
      if (num_right_ticks == 0) {
        PrintIndentation(stream, kResultIndentationNumSpaces);
        stream.println("ERROR: No left encoder ticks detected. Did you turn the left wheel? If so, please check the hardware.");
      } else {
        PrintIndentation(stream, kResultIndentationNumSpaces);
        stream.println("ERROR: Only right encoder ticks detected. Did you turn the wrong wheel? If not, please check the hardware.");
      }
    } else {
      PrintIndentation(stream, kResultIndentationNumSpaces);
      stream.printf("ERROR: %d left encoder ticks detected, while something in [%d, %d] was expected. Did you do one full turn only? If so, please check the hardware.\n", num_left_ticks, kMinTicks, kMaxTicks);
    }
  }

  // Right wheel.
  bool right_ok = false;
  num_left_ticks = 0;
  num_right_ticks = 0;
  stream.printf("2. Spin the RIGHT wheel ONE FULL TURN ONLY in either direction. ");
  WaitForEnterOrTimeout(stream); stream.println();
  if (num_right_ticks >= kMinTicks && num_right_ticks <= kMaxTicks) {
    PrintIndentation(stream, kResultIndentationNumSpaces);
    stream.printf("OK: Right wheel spinned %.2f times.\n", num_right_ticks / kTicksPerWheelTurn);
    right_ok = true;
  } else {
    if (num_right_ticks == 0) {
      if (num_left_ticks == 0) {
        PrintIndentation(stream, kResultIndentationNumSpaces);
        stream.println("ERROR: No right encoder ticks detected. Did you turn the right wheel? If so, please check the hardware.");
      } else {
        PrintIndentation(stream, kResultIndentationNumSpaces);
        stream.println("ERROR: Only left encoder ticks detected. Did you turn the wrong wheel? If not, please check the hardware.");
      }
    } else {
      PrintIndentation(stream, kResultIndentationNumSpaces);
      stream.printf("ERROR: %d right encoder ticks detected, while something in [%d, %d] was expected. Did you do one full turn only? If so, please check the hardware.\n", num_right_ticks, kMinTicks, kMaxTicks);
    }
  }

  if (left_ok && right_ok) {
    stream.println("OK.");
  } else {
    stream.println("ERROR.");
  }

  RemoveEncoderIsrs(&GotLeftEncoderTick, &GotRightEncoderTick);

  return true;
}

static StatusOr<int> SenseMajorAccelerationAxis(Stream &stream) {
  stream.printf("[Press ENTER when done or wait for %.1f seconds]", kDefaultWaitInputTimeoutSeconds);
  // Sample acceleration squared for some time.
  Vector<3> squared_accel_accum(0, 0, 0);
  auto start_seconds = GetTimerSeconds();
  while(!stream.available() && GetTimerSeconds() - start_seconds < kDefaultWaitInputTimeoutSeconds) {
    const auto accel = body_imu.GetLinearAccelerations();
    squared_accel_accum = squared_accel_accum + Vector<3>(accel[0] * accel[0], accel[1] * accel[1], accel[2] * accel[2]);
  }
  // If enter was pressed to end, consume all keys.
  while (stream.available()) {
    stream.read();
  }

  // The shaking should be strong enough.
  constexpr float kMinCumulativeSquaredAccelerationNorm = 1.0f; // [(m/s^2)^2]
  constexpr float kMinAccelerationComponentToNormRatio = 0.7f;
  if (squared_accel_accum.Norm() < kMinCumulativeSquaredAccelerationNorm) {
    return Status::kUnavailableError;
  }

  // Find dominant direction of the cumulative squared acceleration vector.
  int dominant_dimension = 0;
  if (squared_accel_accum.y() > squared_accel_accum[dominant_dimension]) { dominant_dimension = 1; }
  if (squared_accel_accum.z() > squared_accel_accum[dominant_dimension]) { dominant_dimension = 2; }
  if (squared_accel_accum[dominant_dimension] / squared_accel_accum.Norm() < kMinAccelerationComponentToNormRatio * kMinAccelerationComponentToNormRatio) {
    return Status::kMalformedError;
  }

  return dominant_dimension;
}

static bool PrintDirectedShakeTestResult(Stream &stream, StatusOr<int> test_result, int expected_axis_index, int indentation) {
  bool result = false;
  if (test_result.ok()) {
    if (*test_result == expected_axis_index) {
      result = true;
      PrintIndentation(stream, indentation);
      stream.printf("OK: The IMU registered shaking in the %c axis.", 'x' + static_cast<char>(expected_axis_index));
    } else {
      PrintIndentation(stream, indentation);
      stream.printf("ERROR: Shaking detected in the %c axis instead.", 'x' + static_cast<char>(*test_result));
    }
  } else {
    if (test_result.status() == Status::kUnavailableError) {
      PrintIndentation(stream, indentation);
      stream.print("ERROR: Not enough shaking was measured.");
    } else if (test_result.status() == Status::kMalformedError) {
      PrintIndentation(stream, indentation);
      stream.printf("ERROR: Not enough shaking was measured in the %c axis.", 'x' + static_cast<char>(expected_axis_index));
    }
  }
  return result;
}

bool CheckBodyIMU(Stream &stream, bool check_preconditions) {
  if (check_preconditions) {
    stream.println("Running precondition tests...");
    if (!CheckTimer()) {
      stream.println("ERROR: The MCU timer is not working as expected.");
      return false;
    }
  }

  constexpr int kResultIndentationNumSpaces = 3;

  stream.print("1. Put the robot on a flat surface. ");
  WaitForEnterOrTimeout(stream); stream.println();
  const auto ypr0 = body_imu.GetYawPitchRoll();

  constexpr float kExpectedOrientationComponentDiff = M_PI / 2;  
  constexpr float kMaxOrientationComponentDiffError = 0.2 * kExpectedOrientationComponentDiff;
  constexpr float kMinOrientationComponentDiff = kExpectedOrientationComponentDiff - kMaxOrientationComponentDiffError;
  constexpr float kMaxOrientationComponentDiff = kExpectedOrientationComponentDiff + kMaxOrientationComponentDiffError;

  // Yaw test.
  stream.printf("2. Turn the robot %.0f degrees counterclockwise around a vertical axis. ", DegreesFromRadians(kExpectedOrientationComponentDiff));
  WaitForEnterOrTimeout(stream); stream.println();
  const float yaw_diff = body_imu.GetYawPitchRoll().z() - ypr0.z();
  bool yaw_ok = false;
  if (yaw_diff >= kMinOrientationComponentDiff && yaw_diff <= kMaxOrientationComponentDiff) {
    yaw_ok = true;
    PrintIndentation(stream, kResultIndentationNumSpaces);
    stream.printf("OK: The robot's yaw changed %.0f degrees.", DegreesFromRadians(yaw_diff));
  } else {
    PrintIndentation(stream, kResultIndentationNumSpaces);
    stream.printf("ERROR: The robot's yaw changed %.0f degrees, while something in [%.0f, %.0f] was expected.", DegreesFromRadians(yaw_diff), DegreesFromRadians(kMinOrientationComponentDiff), DegreesFromRadians(kMaxOrientationComponentDiff));
  }
  stream.println();

  // Pitch test.
  stream.printf("3. Point the robot's front to the floor. ");
  WaitForEnterOrTimeout(stream); stream.println();
  const float pitch_diff = body_imu.GetYawPitchRoll().y() - ypr0.y();
  bool pitch_ok = false;
  if (pitch_diff >= kMinOrientationComponentDiff && pitch_diff <= kMaxOrientationComponentDiff) {
    pitch_ok = true;
    PrintIndentation(stream, kResultIndentationNumSpaces);
    stream.printf("OK: The robot's pitch changed %.0f degrees.", DegreesFromRadians(pitch_diff));
  } else {
    PrintIndentation(stream, kResultIndentationNumSpaces);
    stream.printf("ERROR: The robot's pitch changed %.0f degrees, while something in [%.0f, %.0f] was expected.", DegreesFromRadians(pitch_diff), DegreesFromRadians(kMinOrientationComponentDiff), DegreesFromRadians(kMaxOrientationComponentDiff));
  }
  stream.println();

  // Roll test.
  stream.printf("4. Lay the robot over the right wheel's side. ");
  WaitForEnterOrTimeout(stream); stream.println();
  const float roll_diff = body_imu.GetYawPitchRoll().x() - ypr0.x();
  bool roll_ok = false;
  if (roll_diff >= kMinOrientationComponentDiff && roll_diff <= kMaxOrientationComponentDiff) {
    roll_ok = true;
    PrintIndentation(stream, kResultIndentationNumSpaces);
    stream.printf("OK: The robot's roll changed %.0f degrees.", DegreesFromRadians(roll_diff));
  } else {
    PrintIndentation(stream, kResultIndentationNumSpaces);
    stream.printf("ERROR: The robot's roll changed %.0f degrees, while something in [%.0f, %.0f] was expected.", DegreesFromRadians(roll_diff), DegreesFromRadians(kMinOrientationComponentDiff), DegreesFromRadians(kMaxOrientationComponentDiff));
  }  
  stream.println();

  // X accelerometer test.
  stream.printf("5. Shake the robot along its front-back direction. ");
  StatusOr<int> maybe_movement_axis = SenseMajorAccelerationAxis(stream); stream.println();
  const bool x_accel_ok = PrintDirectedShakeTestResult(stream, maybe_movement_axis, /*expected_axis_index=*/0, kResultIndentationNumSpaces);
  stream.println();

  // Y accelerometer test.
  stream.printf("6. Shake the robot along its left-right direction. ");
  maybe_movement_axis = SenseMajorAccelerationAxis(stream); stream.println();
  const bool y_accel_ok = PrintDirectedShakeTestResult(stream, maybe_movement_axis, /*expected_axis_index=*/1, kResultIndentationNumSpaces);
  stream.println();

  // Z accelerometer test.
  stream.printf("7. Shake the robot along its up-down direction. ");
  maybe_movement_axis = SenseMajorAccelerationAxis(stream); stream.println();
  const bool z_accel_ok = PrintDirectedShakeTestResult(stream, maybe_movement_axis, /*expected_axis_index=*/2, kResultIndentationNumSpaces);
  stream.println();
  
  if (!yaw_ok || !roll_ok || !pitch_ok || !x_accel_ok || !y_accel_ok || !z_accel_ok) {
    stream.println("ERROR.");
    return false;
  }

  stream.println("OK.");
  return true;
}

static Vector<3> AverageCorrectedAccelerations(TimerSecondsType duration_s, TimerSecondsType accum_start_s) {
  ASSERT(duration_s > 0);
  ASSERT(accum_start_s < duration_s);
  Vector<3> accum(0, 0, 0);
  int num_samples = 0;
  TimerSecondsType elapsed_s = 0;
  constexpr float kReadFrequency = 100.0f; // [Hz]
  const auto start_seconds = GetTimerSeconds();
  TimerSecondsType last_sample_seconds = -std::numeric_limits<float>::infinity();
  while(true) {
    const TimerSecondsType now = GetTimerSeconds();
    elapsed_s = now - start_seconds;    
    if (elapsed_s >= accum_start_s) {      
      if (now - last_sample_seconds < 1.0f / kReadFrequency) {
        continue;
      }
      last_sample_seconds = now;
      const auto lin_accel = body_imu.GetLinearAccelerations();
      const auto ypr = body_imu.GetYawPitchRoll();
      // The linear acceleration estimate is not great for the y axis with the sort of movement 
      // used in our circular motion checks, but it seems correctable if we compensante for the 
      // part of the gravity vector projected on the y axis due to pitch.
      const auto accel_minus_gravity = Vector<3>(lin_accel.x(), lin_accel.y() + 9.81f * cosf(M_PI / 2 - ypr.x()) / cosf(ypr.x()), lin_accel.z());
      accum = accum + accel_minus_gravity;
      ++num_samples;
    }
    if (elapsed_s >= duration_s) {
      break;
    }
  }
  return accum / num_samples;
}

constexpr float kCheckBodyMotionBaseSpinSeconds = 1.0f;

static bool CheckBodyMotionParamsAfterWheelRotation(Stream &stream, const Vector<3> &ypr0, const Vector<3> &start_average_acceleration, const Vector<3> &end_average_acceleration, bool is_clockwise) {
  // Steady state determined empirically for 0.7 PWM duty cycle is approx. 5 turns in 10s.
  constexpr float kMinExpectedAngularSpeed = 3.5 * (2 * M_PI) / 10.0;
  constexpr float kMaxExpectedAngularSpeed = 6.5 * (2 * M_PI) / 10.0;

  constexpr float kMinExpectedWheelTurns = (kMinExpectedAngularSpeed * kCheckBodyMotionBaseSpinSeconds * kRobotDistanceBetweenTireCenters) / (2 * M_PI * kWheelRadius);
  constexpr float kMaxExpectedWheelTurns = (kMaxExpectedAngularSpeed * kCheckBodyMotionBaseSpinSeconds * kRobotDistanceBetweenTireCenters) / (2 * M_PI * kWheelRadius);
  constexpr int kMinExpectedActiveWheelTicks = static_cast<int>((kMinExpectedWheelTurns * 2 * M_PI) / kRadiansPerWheelTick);
  constexpr int kMaxExpectedActiveWheelTicks = static_cast<int>(ceilf((kMaxExpectedWheelTurns * 2 * M_PI) / kRadiansPerWheelTick));
  constexpr int kMaxExpectedInactiveWheelTicks = ((2 * M_PI) / kRadiansPerWheelTick) * 0.1;
  bool num_active_encoder_ticks_ok = false;
  bool num_inactive_encoder_ticks_ok = false;
  if (is_clockwise) {
    // The right wheel turned backwards.
    if (num_right_ticks >= kMinExpectedActiveWheelTicks && num_right_ticks <= kMaxExpectedActiveWheelTicks) {
      num_active_encoder_ticks_ok = true;
      stream.printf("OK: Got %d ticks from the right encoder, which is within the expected range [%d, %d].\n", num_right_ticks, kMinExpectedActiveWheelTicks, kMaxExpectedActiveWheelTicks);
    } else {
      stream.printf("ERROR: Got %d ticks from the right encoder, whi  h is NOT within the expected range [%d, %d].\n", num_right_ticks, kMinExpectedActiveWheelTicks, kMaxExpectedActiveWheelTicks);
    }
    if (num_left_ticks <= kMaxExpectedInactiveWheelTicks) {
      num_inactive_encoder_ticks_ok = true;
      stream.printf("OK: Got %d ticks from the left encoder, which is within the expected range [0, %d].\n", num_left_ticks, kMaxExpectedInactiveWheelTicks);
    } else {
      stream.printf("ERROR: Got %d ticks from the left encoder, which is NOT within the expected range [0, %d].\n", num_left_ticks, kMaxExpectedInactiveWheelTicks);
    }
  } else {
    // The left wheel turned backwards.
    if (num_left_ticks >= kMinExpectedActiveWheelTicks && num_left_ticks <= kMaxExpectedActiveWheelTicks) {
      num_active_encoder_ticks_ok = true;
      stream.printf("OK: Got %d ticks from the left encoder, which is within the expected range [%d, %d].\n", num_left_ticks, kMinExpectedActiveWheelTicks, kMaxExpectedActiveWheelTicks);
    } else {
      stream.printf("ERROR: Got %d ticks from the left encoder, which is NOT within the expected range [%d, %d].\n", num_left_ticks, kMinExpectedActiveWheelTicks, kMaxExpectedActiveWheelTicks);
    }
    if (num_right_ticks <= kMaxExpectedInactiveWheelTicks) {
      num_inactive_encoder_ticks_ok = true;
      stream.printf("OK: Got %d ticks from the right encoder, which is within the expected range [0, %d].\n", num_right_ticks, kMaxExpectedInactiveWheelTicks);
    } else {
      stream.printf("ERROR: Got %d ticks from the right encoder, which is NOT within the expected range [0, %d].\n", num_right_ticks, kMaxExpectedInactiveWheelTicks);
    }
  }

  // Check backward acceleration due to starting to move backwards.
  constexpr float kMinExpectedTimeToReachAngularSpeed = 0.25;
  constexpr float kMaxExpectedTimeToReachAngularSpeed = 1.0;
  constexpr float kMaxExpectedForwardAcceleration = -(kMinExpectedAngularSpeed * kRobotDistanceBetweenTireCenters) / kMaxExpectedTimeToReachAngularSpeed;
  constexpr float kMinExpectedForwardAcceleration = -(kMaxExpectedAngularSpeed * kRobotDistanceBetweenTireCenters) / kMinExpectedTimeToReachAngularSpeed;
  bool accel_x_ok = false;
  if (start_average_acceleration.x() >= kMinExpectedForwardAcceleration && start_average_acceleration.x() <= kMaxExpectedForwardAcceleration) {
    accel_x_ok = true;
    stream.printf("OK: The forward acceleration of %.3f m/s^2 is within the expected interval [%.3f, %.3f] m/s^2.\n", start_average_acceleration.x(), kMinExpectedForwardAcceleration, kMaxExpectedForwardAcceleration);
  } else {
    stream.printf("ERROR: The forward acceleration of %.3f m/s^2 is NOT within the expected interval [%.3f, %.3f] m/s^2.\n", start_average_acceleration.x(), kMinExpectedForwardAcceleration, kMaxExpectedForwardAcceleration);
  }

  // Check centripetal acceleration due to one wheel rotation.
  constexpr float kMinExpectedCentripetalAccel = (kMinExpectedAngularSpeed * kMinExpectedAngularSpeed) * (kRobotDistanceBetweenTireCenters / 2);
  constexpr float kMaxExpectedCentripetalAccel = (kMaxExpectedAngularSpeed * kMaxExpectedAngularSpeed) * (kRobotDistanceBetweenTireCenters / 2);
  const float sign = is_clockwise ? 1.0f : -1.0f;
  const float min_accel_y = sign * (is_clockwise ? kMinExpectedCentripetalAccel : kMaxExpectedCentripetalAccel);
  const float max_accel_y = sign * (is_clockwise ? kMaxExpectedCentripetalAccel : kMinExpectedCentripetalAccel);
  bool accel_y_ok = false;
  if (end_average_acceleration.y() >= min_accel_y && end_average_acceleration.y() <= max_accel_y) {
    accel_y_ok = true;
    stream.printf("OK: The centripetal acceleration of %.3f m/s^2 is within the expected interval [%.3f, %.3f] m/s^2.\n", end_average_acceleration.y(), min_accel_y, max_accel_y);
  } else {
    stream.printf("ERROR: The centripetal acceleration of %.3f m/s^2 is NOT within the expected interval [%.3f, %.3f] m/s^2.\n", end_average_acceleration.y(), min_accel_y, max_accel_y);
  }

  // Check orientation change due to one wheel rotation.
  // The actual change could be lower because of the wheel acceleration transient; therefore, the factor < 1.
  constexpr float kMinExpectedYawChange = -0.7 * kMinExpectedAngularSpeed * kCheckBodyMotionBaseSpinSeconds;  // [rad]
  constexpr float kMaxExpectedYawChange = -0.95 * kMaxExpectedAngularSpeed * kCheckBodyMotionBaseSpinSeconds;  // [rad]
  const float min_yaw_change = sign * (is_clockwise ? kMaxExpectedYawChange : kMinExpectedYawChange);
  const float max_yaw_change = sign * (is_clockwise ? kMinExpectedYawChange : kMaxExpectedYawChange);
  constexpr float kMaxExpectedNonYawAbsoluteChange = 10 * M_PI / 180;
  bool yaw_diff_ok = false;
  auto ypr = body_imu.GetYawPitchRoll();
  const float yaw_diff = NormalizeRadians(ypr.z() - ypr0.z());
  if (yaw_diff >= min_yaw_change && yaw_diff <= max_yaw_change) {
    yaw_diff_ok = true;
    stream.printf("OK: The yaw change of %.1f degress is within the expected interval [%.1f, %.1f] degrees.\n", DegreesFromRadians(yaw_diff), DegreesFromRadians(min_yaw_change), DegreesFromRadians(max_yaw_change));
  } else {
    stream.printf("ERROR: The yaw change of %.1f degress is NOT within the expected interval [%.1f, %.1f] degrees.\n", DegreesFromRadians(yaw_diff), DegreesFromRadians(min_yaw_change), DegreesFromRadians(max_yaw_change));
  }
  bool pitch_diff_ok = false;
  const float pitch_diff = NormalizeRadians(ypr.y() - ypr0.y());  
  if (fabsf(pitch_diff) < kMaxExpectedNonYawAbsoluteChange) {
    pitch_diff_ok = true;
    stream.printf("OK: The pitch change of %.1f degress is within the expected interval [%.1f, %.1f] degrees.\n", DegreesFromRadians(pitch_diff), DegreesFromRadians(-kMaxExpectedNonYawAbsoluteChange), DegreesFromRadians(kMaxExpectedNonYawAbsoluteChange));
  } else {
    stream.printf("ERROR: The pitch change of %.1f degress is NOT within the expected interval [%.1f, %.1f] degrees.\n", DegreesFromRadians(pitch_diff), DegreesFromRadians(-kMaxExpectedNonYawAbsoluteChange), DegreesFromRadians(kMaxExpectedNonYawAbsoluteChange));
  }
  bool roll_diff_ok = false;
  const float roll_diff = NormalizeRadians(ypr.x() - ypr0.x());  
  if (fabsf(roll_diff) < kMaxExpectedNonYawAbsoluteChange) {
    roll_diff_ok = true;
    stream.printf("OK: The roll change of %.1f degress is within the expected interval [%.1f, %.1f] degrees.\n", DegreesFromRadians(roll_diff), DegreesFromRadians(-kMaxExpectedNonYawAbsoluteChange), DegreesFromRadians(kMaxExpectedNonYawAbsoluteChange));
  } else {
    stream.printf("ERROR: The roll change of %.1f degress is NOT within the expected interval [%.1f, %.1f] degrees.\n", DegreesFromRadians(roll_diff), DegreesFromRadians(-kMaxExpectedNonYawAbsoluteChange), DegreesFromRadians(kMaxExpectedNonYawAbsoluteChange));
  }

  return num_active_encoder_ticks_ok && num_inactive_encoder_ticks_ok && accel_x_ok && accel_y_ok && yaw_diff_ok && roll_diff_ok && pitch_diff_ok;
}

bool CheckBodyMotion(Stream &stream, bool check_preconditions) {
  if (check_preconditions) {
    stream.println("Running precondition tests...");
    if (!CheckTimer()) {
      stream.println("ERROR: The MCU timer is not working as expected.");
      return false;
    }
    if (!CheckBattery()) {
      stream.println("ERROR: Battery voltage is to low to run this test.");
      return false;
    }
  }

  const bool old_wheel_control_status = EnableWheelControl(false);
  const bool old_trajectory_control_status = EnableTrajectoryControl(false);

  num_left_ticks = 0;
  num_right_ticks = 0;
  AddEncoderIsrs(&GotLeftEncoderTick, &GotRightEncoderTick);

  // Move left wheel.
  num_left_ticks = 0;
  num_right_ticks = 0;
  auto ypr0 = body_imu.GetYawPitchRoll();  
  stream.printf("Spinning left motor backward for %.1f seconds...\n", kCheckBodyMotionBaseSpinSeconds);
  SetLeftMotorDutyCycle(-0.7);

  auto start_average_acceleration = AverageCorrectedAccelerations(kCheckBodyMotionBaseSpinSeconds * 0.25f, 0);
  auto end_average_acceleration = AverageCorrectedAccelerations(kCheckBodyMotionBaseSpinSeconds * 0.75f, kCheckBodyMotionBaseSpinSeconds * 0.75f * 0.75f);
  const bool left_ok = CheckBodyMotionParamsAfterWheelRotation(stream, ypr0, start_average_acceleration, end_average_acceleration, /*is_clockwise=*/false);

  stream.printf("Spinning left motor forward for %.1f seconds...\n", kCheckBodyMotionBaseSpinSeconds);
  SetLeftMotorDutyCycle(0.7);
  SleepForSeconds(kCheckBodyMotionBaseSpinSeconds);
  SetLeftMotorDutyCycle(0);

  SleepForSeconds(1.0f);

  // Move right wheel.
  num_left_ticks = 0;
  num_right_ticks = 0;
  ypr0 = body_imu.GetYawPitchRoll();  
  stream.printf("Spinning right motor backward for %.1f seconds...\n", kCheckBodyMotionBaseSpinSeconds);
  SetRightMotorDutyCycle(-0.7);

  start_average_acceleration = AverageCorrectedAccelerations(kCheckBodyMotionBaseSpinSeconds * 0.25f, 0);
  end_average_acceleration = AverageCorrectedAccelerations(kCheckBodyMotionBaseSpinSeconds * 0.75f, kCheckBodyMotionBaseSpinSeconds * 0.75f * 0.75f);
  const bool right_ok = CheckBodyMotionParamsAfterWheelRotation(stream, ypr0, start_average_acceleration, end_average_acceleration, /*is_clockwise=*/true);

  stream.printf("Spinning right motor forward for %.1f seconds...\n", kCheckBodyMotionBaseSpinSeconds);
  SetRightMotorDutyCycle(0.7);
  SleepForSeconds(kCheckBodyMotionBaseSpinSeconds);
  SetRightMotorDutyCycle(0);

  // Clean up.
  RemoveEncoderIsrs(&GotLeftEncoderTick, &GotRightEncoderTick);
  EnableWheelControl(old_wheel_control_status);
  EnableTrajectoryControl(old_trajectory_control_status);

  if (!left_ok || !right_ok) {
    stream.println("ERROR.");
    return false;
  } 

  stream.println("OK.");
  return true;
}
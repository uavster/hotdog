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

#define kMinBatteryVoltage 7.0

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
            stream.printf("SRAM is damaged at %p.", bad_address);
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
        stream.printf("SRAM is damaged at %p.", bp);
        return false;
      }
      *bp = old_value;
    }

    stream.printf("OK.");
  }
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

  // Move each wheel backward and forward.
  constexpr float kAtomicMotionSeconds = 0.5f;
  stream.printf("Spinning left wheel for %.1f seconds...\n", kAtomicMotionSeconds);
  SetLeftMotorDutyCycle(-0.7);
  SleepForSeconds(kAtomicMotionSeconds);
  SetLeftMotorDutyCycle(0.7);
  SleepForSeconds(kAtomicMotionSeconds);
  SetLeftMotorDutyCycle(0);

  stream.printf("Spinning right wheel for %.1f seconds...\n", kAtomicMotionSeconds);
  SetRightMotorDutyCycle(-0.7);
  SleepForSeconds(kAtomicMotionSeconds);
  SetRightMotorDutyCycle(0.7);
  SleepForSeconds(kAtomicMotionSeconds);
  SetRightMotorDutyCycle(0);

  stream.printf("If the wheels did not spin back and forth one at a time -first left, then right- for %.1f seconds each, check the hardware.\n", kAtomicMotionSeconds);

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
  constexpr float kTimeoutSeconds = 8.0f;
  constexpr float kTicksPerWheelTurn = 2 * M_PI / kRadiansPerWheelTick;
  constexpr float kMaxErrorNumTurns = 0.2;
  constexpr int kMaxErrorTicks = static_cast<int>(kMaxErrorNumTurns * kTicksPerWheelTurn);
  constexpr int kMinTicks = kTicksPerWheelTurn - kMaxErrorTicks;
  constexpr int kMaxTicks = kTicksPerWheelTurn + kMaxErrorTicks;

  // Left wheel.
  bool left_ok = false;
  stream.printf("Please spin the LEFT wheel ONE FULL TURN ONLY in either direction. I'll wait for %.1f seconds.\n", kTimeoutSeconds);
  SleepForSeconds(kTimeoutSeconds);
  if (num_left_ticks >= kMinTicks && num_left_ticks <= kMaxTicks) {
    stream.printf("OK: Left wheel spinned %.2f times.\n", num_left_ticks / kTicksPerWheelTurn);
    left_ok = true;
  } else {
    if (num_left_ticks == 0) {
      if (num_right_ticks == 0) {
        stream.println("ERROR: No left encoder ticks detected.\nDid you turn the left wheel? If so, please check the hardware.");
      } else {
        stream.println("ERROR: Only right encoder ticks detected.\nDid you turn the wrong wheel? If not, please check the hardware.");
      }
    } else {
      stream.printf("ERROR: %d left encoder ticks detected, while something in [%d, %d] was expected.\nDid you do one full turn only? If so, please check the hardware.\n", num_left_ticks, kMinTicks, kMaxTicks);
    }
  }

  // Right wheel.
  bool right_ok = false;
  num_left_ticks = 0;
  num_right_ticks = 0;
  stream.printf("Please spin the RIGHT wheel ONE FULL TURN in either direction. I'll wait for %.1f seconds.\n", kTimeoutSeconds);
  SleepForSeconds(kTimeoutSeconds);
  if (num_right_ticks >= kMinTicks && num_right_ticks <= kMaxTicks) {
    stream.printf("OK: Right wheel spinned %.2f times.\n", num_right_ticks / kTicksPerWheelTurn);
    right_ok = true;
  } else {
    if (num_right_ticks == 0) {
      if (num_left_ticks == 0) {
        stream.println("ERROR: No right encoder ticks detected.\nDid you turn the right wheel? If so, please check the hardware.");
      } else {
        stream.println("ERROR: Only left encoder ticks detected.\nDid you turn the wrong wheel? If not, please check the hardware.");
      }
    } else {
      stream.printf("ERROR: %d right encoder ticks detected, while something in [%d, %d] was expected.\nDid you do one full turn only? If so, please check the hardware.\n", num_right_ticks, kMinTicks, kMaxTicks);
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

bool CheckIMU(Stream &stream, bool check_preconditions) {
  if (check_preconditions) {
    stream.println("Running precondition tests...");
    if (!CheckTimer()) {
      stream.println("ERROR: The MCU timer is not working as expected.");
      return false;
    }
  }

  constexpr float kTimeoutSeconds = 8.0f;

  // Yaw test.
  constexpr float kExpectedYawDiff = M_PI / 2;  
  auto ypr0 = body_imu.GetYawPitchRoll();
  stream.printf("Make sure the robot lies on a flat surface and turn it %.0f degrees counterclockwise around a vertical axis. I'll wait for %.1f seconds.\n", DegreesFromRadians(kExpectedYawDiff), kTimeoutSeconds);
  SleepForSeconds(kTimeoutSeconds);
  auto ypr = body_imu.GetYawPitchRoll();
  const float yaw_diff = ypr.z() - ypr0.z();
  constexpr float kMaxYawDiffError = 0.2 * kExpectedYawDiff;
  constexpr float kMinYawDiff = kExpectedYawDiff - kMaxYawDiffError;
  constexpr float kMaxYawDiff = kExpectedYawDiff + kMaxYawDiffError;
  bool yaw_ok = false;
  if (yaw_diff >= kMinYawDiff && yaw_diff <= kMaxYawDiff) {
    yaw_ok = true;
    stream.printf("OK: The robot's yaw changed %.0f degrees.\n", DegreesFromRadians(yaw_diff));
  } else {
    stream.printf("ERROR: The robot's yaw changed %.0f degrees, while something in [%.0f, %.0f] was expected.\n", DegreesFromRadians(yaw_diff), DegreesFromRadians(kMinYawDiff), DegreesFromRadians(kMaxYawDiff));
  }

  // Pitch test.
  constexpr float kExpectedPitchDiff = M_PI / 2;  
  ypr0 = body_imu.GetYawPitchRoll();
  stream.printf("Now, point the robot's front to the floor. I'll wait for %.1f seconds.\n", DegreesFromRadians(kExpectedPitchDiff), kTimeoutSeconds);
  SleepForSeconds(kTimeoutSeconds);
  ypr = body_imu.GetYawPitchRoll();
  const float pitch_diff = ypr.y() - ypr0.y();
  constexpr float kMaxPitchDiffError = 0.2 * kExpectedPitchDiff;
  constexpr float kMinPitchDiff = kExpectedPitchDiff - kMaxPitchDiffError;
  constexpr float kMaxPitchDiff = kExpectedPitchDiff + kMaxPitchDiffError;
  bool pitch_ok = false;
  if (pitch_diff >= kMinPitchDiff && pitch_diff <= kMaxPitchDiff) {
    pitch_ok = true;
    stream.printf("OK: The robot's pitch changed %.0f degrees.\n", DegreesFromRadians(pitch_diff));
  } else {
    stream.printf("ERROR: The robot's pitch changed %.0f degrees, while something in [%.0f, %.0f] was expected.\n", DegreesFromRadians(pitch_diff), DegreesFromRadians(kMinPitchDiff), DegreesFromRadians(kMaxPitchDiff));
  }

  // Roll test.
  constexpr float kExpectedRollDiff = M_PI / 2;  
  ypr0 = body_imu.GetYawPitchRoll();
  stream.printf("Now, lay the robot over the right wheel's side. I'll wait for %.1f seconds.\n", kTimeoutSeconds);
  SleepForSeconds(kTimeoutSeconds);
  ypr = body_imu.GetYawPitchRoll();
  const float roll_diff = ypr.x() - ypr0.x();
  constexpr float kMaxRollDiffError = 0.2 * kExpectedRollDiff;
  constexpr float kMinRollDiff = kExpectedRollDiff - kMaxRollDiffError;
  constexpr float kMaxRollDiff = kExpectedRollDiff + kMaxRollDiffError;
  bool roll_ok = false;
  if (roll_diff >= kMinRollDiff && roll_diff <= kMaxRollDiff) {
    roll_ok = true;
    stream.printf("OK: The robot's roll changed %.0f degrees.\n", DegreesFromRadians(roll_diff));
  } else {
    stream.printf("ERROR: The robot's roll changed %.0f degrees, while something in [%.0f, %.0f] was expected.\n", DegreesFromRadians(roll_diff), DegreesFromRadians(kMinRollDiff), DegreesFromRadians(kMaxRollDiff));
  }  

  // Get acceleration offsets.
  imu::Vector<3> accel_offsets;
  int num_samples = 0;
  auto start_seconds = GetTimerSeconds();
  while(GetTimerSeconds() - start_seconds < 1.0) {
    accel_offsets = accel_offsets + body_imu.GetLinearAccelerations();
  }
  accel_offsets = accel_offsets / num_samples;

  constexpr float kMinAccelerationComponentToMagnitudeRatio = 0.7f;
  constexpr float kMinAccelerationMagnitude = 1.0f; // [m/s^2]

  // X accelerometer test.
  stream.printf("Now, shake the robot along the front-back direction. I'll wait for %.1f seconds.\n", kTimeoutSeconds);
  start_seconds = GetTimerSeconds();
  imu::Vector<3> squared_accel_accum;
  while(GetTimerSeconds() - start_seconds < kTimeoutSeconds) {
    const auto accel = body_imu.GetLinearAccelerations() - accel_offsets;
    squared_accel_accum = squared_accel_accum + imu::Vector<3>(accel[0] * accel[0], accel[1] * accel[1], accel[2] * accel[2]);
  }
  // Check if the x component is the biggest contributor to the magnitude.
  bool x_accel_ok = false;
  if (squared_accel_accum.magnitude() > kMinAccelerationMagnitude) {
    if (squared_accel_accum.x() / squared_accel_accum.magnitude() >= kMinAccelerationComponentToMagnitudeRatio) {
      x_accel_ok = true;
      stream.println("OK: The IMU registered most of the acceleration in the x axis.");
    } else {
      stream.println("ERROR: The x axis is not receiving most of the acceleration.");
    }
  } else {
    stream.println("ERROR: Not enough acceleration was measured.");
  }
  
  if (!yaw_ok || !roll_ok || !pitch_ok || !x_accel_ok) {
    stream.println("ERROR.");
    return false;
  }

  stream.println("OK.");
  return true;
}
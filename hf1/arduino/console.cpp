#include "kinetis.h"
#include "console.h"
#include "battery.h"
#include "status_or.h"
#include "body_imu.h"
#include "motors.h"
#include "checks.h"
#include "servos.h"

#include "timer_arduino.h"
#include "wheel_controller.h"
#include "wheel_state_estimator.h"
#include "operation_mode.h"

extern TimerArduino timer;
extern WheelSpeedController left_wheel;
extern WheelSpeedController right_wheel;
extern WheelStateEstimator wheel_state_estimator;

namespace {
const char *SkipSpaces(const char *str) {  
  while (*str == ' ' && *str != '\0') { ++str; }
  return str;
}

const char *SkipNonSpaces(const char *str) {
  while (*str != ' ' && *str != '\0') { ++str; }
  return str;
}

bool IsDigit(char c) {
  return c >= '0' && c <= '9';
}

StatusOr<TimerNanosType> NanosFromDurationString(const StringView &s) {
  const StringView trimmed_s = s.Trimmed();
  if (trimmed_s.Length() == 0) { return Status::kMalformedError; }
  StringView units = trimmed_s;
  while (IsDigit(*units.start) && units.start < units.end) { ++units.start; }
  StringView number;
  number.start = trimmed_s.start;
  number.end = units.start;
  char number_str[number.Length() + 1];
  number.ToCString(number_str);
  StatusOr<TimerNanosType> period_ns = Status::kMalformedError;
  double period;
  if (sscanf(number_str, "%lf", &period) == 1) {
    if (units.Length() == 0 || units == "s") {    
      period_ns = period * 1'000'000'000ULL;
    } else if (units == "ms") {
      period_ns = period * 1'000'000;
    }  else if (units == "us") {
      period_ns = period * 1'000;
    }  else if (units == "ns") {
      period_ns = period;
    }
  }
  return period_ns;
}

void PrintBodyIMUCalibrationStatus(Stream &stream, const BodyIMU::CalibrationStatus &status) {
  static const char done_str[] = " (done)";
  static const char incomplete_str[] = "";
  static const char yes_str[] = "YES";
  static const char no_str[] = "NO";
  stream.printf("Body IMU calibrated: %s ; Details: system=%d%s gyroscopes=%d%s accelerometers=%d%s magnetometer=%d%s\n", 
    status.IsFullyCalibrated() ? yes_str : no_str,
    status.system, status.IsSystemCalibrated() ? done_str : incomplete_str, 
    status.gyroscopes, status.AreGyroscopesCalibrated() ? done_str : incomplete_str, 
    status.accelerometers, status.AreAccelerometersCalibrated() ? done_str : incomplete_str, 
    status.magnetometer, status.IsMagnetometerCalibrated() ? done_str : incomplete_str);
}

void PrintBodyIMUCalibrationData(Stream &stream, const BodyIMU::CalibrationData &data) {
  stream.printf("[Accelerometers]\n  Offsets: x=%d y=%d z=%d\n  Radius: %d\n[Gyroscopes]  Offsets: x=%d y=%d z=%d\n[Magnetometer]\n  Offsets: x=%d y=%d z=%d\n  Radius: %d\n", 
    data.accel_offset_x, data.accel_offset_y, data.accel_offset_z, data.accel_radius,
    data.gyro_offset_x, data.gyro_offset_y, data.gyro_offset_z, 
    data.mag_offset_x, data.mag_offset_y, data.mag_offset_z, data.mag_radius
  );
}

} // namespace

CommandLine CommandLine::ShiftLeft() const {
  CommandLine new_line;
  new_line.command_name = params[0];
  new_line.num_params = num_params - 1;
  for (int i = 0; i < new_line.num_params; ++i) {
    new_line.params[i] = params[i + 1];
  }
  return new_line;
}

CommandLine CommandLine::DeepCopy(char *dest_buffer) const {
  char *p = dest_buffer;
  CommandLine output;
  command_name.ToCString(p);
  output.command_name.start = p;
  p += command_name.Length();
  output.command_name.end = p;
  ++p;
  output.num_params = num_params;
  for (int i = 0; i < num_params; ++i) {
    params[i].ToCString(p);
    output.params[i].start = p;
    p += params[i].Length();
    output.params[i].end = p;
    ++p;
  }
  return output;
}

void Console::Run() {
  periodic_command_.Run();
  while (input_stream_.available()) {
    const int c = input_stream_.read();
    if (c < 0) { return; }
    if (c == '\n' || c == '\r' || c == '\0' || command_line_length_ >= sizeof(command_line_) - 1) {
      periodic_command_.period_nanos(kPeriodicRunnableInfinitePeriod);
      command_line_[min(command_line_length_, sizeof(command_line_) - 1)] = '\0';
      ProcessCommandLine();
      command_line_length_ = 0;
    } else {
      command_line_[command_line_length_++] = c;
    }
  }
}

CommandHandler *CommandInterpreter::FindCommandHandler(const StringView &command_name) const {
  for (int i = 0; i < num_command_handlers_; ++i) {
    if (!command_handlers_[i]->Matches(command_name)) {
      continue;
    }
    return command_handlers_[i];
  }
  return nullptr;
}

void CommandInterpreter::PrintCommandDescriptions(Stream &stream, const CommandLine &command_line) const {
  for (int i = 0; i < num_command_handlers(); ++i) {
    stream.printf("%s - ", command_handlers()[i]->name());
    command_handlers()[i]->Describe(stream, command_line);
  }
}

static void ShowCommandHelpRecursive(Stream &stream, const CommandInterpreter &interpreter, const CommandLine &command_line) {
  ASSERT(command_line.num_params > 0);
  CommandHandler *handler = interpreter.FindCommandHandler(command_line.params[0]);
  if (handler == nullptr) { 
    stream.printf("Wrong argument '");
    command_line.params[0].Print(stream);
    stream.println("'.");
    return;
  }
  if (command_line.num_params < 2) {
    handler->Help(stream, command_line);
  } else {
    if (handler->interpreter() != nullptr) {
      ShowCommandHelpRecursive(stream, *handler->interpreter(), command_line.ShiftLeft());
    } else {
      stream.printf("'");
      command_line.params[0].Print(stream);
      stream.println("' takes no extra commands.");
    }
  }
}

void HelpCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  if (command_line.num_params == 0) {
    stream.println("Available commands:");
    interpreter_.PrintCommandDescriptions(stream, command_line);
  } else {
    // First parameter is the command name the user needs help about.
    ShowCommandHelpRecursive(stream, interpreter_, command_line);
  }
}

void HelpCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Prints usage information about available commands.");
}

void VersionCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  stream.println("HF1 v0.1");
  stream.println("(c) Ignacio Mellado Bataller");
}

void VersionCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Prints the software version.");
}

void CategoryHandler::Run(Stream &stream, const CommandLine &command_line) {
  CommandHandler *handler = nullptr;
  static const char empty_string[] = "";
  StringView arg0(empty_string);
  if (command_line.num_params >= 1) {
    arg0 = command_line.params[0];
  }
  handler = interpreter_.FindCommandHandler(arg0);
  if (handler == nullptr) {
    stream.print("Wrong argument '");
    command_line.params[0].Print(stream);
    stream.print("'. Please use one of these: ");
  }
  if (handler == nullptr) {
    for (int i = 0; i < interpreter_.num_command_handlers(); ++i) {
      if (i > 0) {
        stream.print(", ");
      }
      stream.print(interpreter_.command_handlers()[i]->name());
    }
    stream.println(".");
    return;
  }

  // Run the subcommand taking as first parameter the one behind the subcommand's name.
  handler->Run(stream, command_line.ShiftLeft());
}

void CategoryHandler::Help(Stream &stream, const CommandLine &command_line) {
  Describe(stream, command_line);
  stream.println("Valid arguments after this one:");
  interpreter_.PrintCommandDescriptions(stream, command_line);
}

void ReadBatteryCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  stream.printf("%.1fV\n", GetBatteryVoltage());
}

void ReadBatteryCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Prints the battery voltage.");
}

void ReadBodyIMUOrientationCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  const auto ypr = body_imu.GetYawPitchRoll();
  stream.printf("yaw:%f, pitch:%f, roll:%f [radians]\n", ypr.z(), ypr.y(), ypr.x());
}

void ReadBodyIMUOrientationCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Orientation in yaw, pitch, roll euler angles, in radians.");
}

void ReadBodyIMUOrientationCommandHandler::Help(Stream &stream, const CommandLine &command_line) {
  Describe(stream, command_line);
  stream.println("Every eurler angle is rotation around an axis of the reference frame, following the right hand rule.");
  stream.println("The reference frame is located at the center of the robot.");
  stream.println("The x axis points to robot's front.");
  stream.println("The y axis points to robot's left.");
  stream.println("The z axis points to sky.");
}

void ReadBodyIMUAccelerationCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  const auto acceleration = body_imu.GetLinearAccelerations();
  stream.printf("(%f, %f, %f) m/s^2\n", acceleration.x(), acceleration.y(), acceleration.z());
}

void ReadBodyIMUAccelerationCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Acceleration vector: (x, y, z) m/s^2.");
}

void ReadBodyIMUAccelerationCommandHandler::Help(Stream &stream, const CommandLine &command_line) {
  Describe(stream, command_line);
  stream.println("The reference frame is located at the center of the robot.");
  stream.println("The x axis points to robot's front.");
  stream.println("The y axis points to robot's left.");
  stream.println("The z axis points to sky.");
}

void ReadTimerTicksCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  char tmp[21];
  Uint64ToString(GetTimerTicks(), tmp);
  stream.printf("%s\n", tmp);
}

void ReadTimerTicksCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Reads the local timer ticks elapsed since boot.");
}

void ReadTimerUnitsCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  char tmp[21];
  const auto now = timer.GetLocalNanoseconds();
  Uint64ToString(now / nanos_to_units_divisor_, tmp);
  stream.printf("%s.", tmp);
  Uint64ToString(now % nanos_to_units_divisor_, tmp);
  stream.printf("%s\n", tmp);
}

void ReadTimerUnitsCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.printf("Reads the local timer %s elapsed since boot.\n", units_name_);
}

void ReadGlobalTimerUnitsCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  char tmp[21];
  const auto now = timer.GetGlobalNanoseconds();
  Uint64ToString(now / nanos_to_units_divisor_, tmp);
  stream.printf("%s.", tmp);
  Uint64ToString(now % nanos_to_units_divisor_, tmp);
  stream.printf("%s\n", tmp);
}

void ReadGlobalTimerUnitsCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.printf("Reads the global timer %s elapsed since boot.\n", units_name_);
}

void ReadEncodersTicksCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  char left_ticks_str[21];
  const StatusOr<TimerTicksType> maybe_left_ticks = wheel_state_estimator.left_wheel_state_filter().last_encoder_edge_timer_ticks();
  if (maybe_left_ticks.ok()) {
    Uint64ToString(*maybe_left_ticks, left_ticks_str);
  } else {
    left_ticks_str[0] = '-';
    left_ticks_str[1] = '\0';
  }
  char right_ticks_str[21];
  const StatusOr<TimerTicksType> maybe_right_ticks = wheel_state_estimator.right_wheel_state_filter().last_encoder_edge_timer_ticks();
  if (maybe_right_ticks.ok()) {
    Uint64ToString(*maybe_right_ticks, right_ticks_str);
  } else {
    right_ticks_str[0] = '-';
    right_ticks_str[1] = '\0';
  }
  stream.printf("%s, %s\n", left_ticks_str, right_ticks_str);
}

void ReadEncodersTicksCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Reads the timestamps of the last encoder ticks.");
}

void ReadEncodersLinearSpeedCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  stream.printf("%f, %f m/s\n", wheel_state_estimator.left_wheel_state_filter().state().speed(), wheel_state_estimator.right_wheel_state_filter().state().speed());
}

void ReadEncodersLinearSpeedCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Reads the linear speed of the wheels estimated with the encoders.");
}

void ReadBodyIMUCalibrationDataCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  PrintBodyIMUCalibrationData(stream, body_imu.GetCalibrationData());
}

void ReadBodyIMUCalibrationDataCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Reads the calibration data of the body IMU.");
}

void ReadBodyIMUCalibrationStatusCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  PrintBodyIMUCalibrationStatus(stream, body_imu.GetCalibrationStatus());
}

void ReadBodyIMUCalibrationStatusCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Reads the calibration status of different subsystems of the body IMU.");
}

void WriteMotorsPWMCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  if (command_line.num_params != 2) {
    stream.println("This command takes two floating point arguments in [0, 1]: write motors pwm left_pwm_duty_cycle right_pwm_duty_cycle.");
    return;
  }
  const StatusOr<float> left_pwm = command_line.params[0].ToFloat();
  if (!left_pwm.ok()) {
    stream.println("Unable to parse left motor's PWM duty cycle.");
    return;
  }
  const StatusOr<float> right_pwm = command_line.params[1].ToFloat();
  if (!right_pwm.ok()) {
    stream.println("Unable to parse right motor's PWM duty cycle.");
    return;
  }
  SetLeftMotorDutyCycle(*left_pwm);
  SetRightMotorDutyCycle(*right_pwm);
  if (EnableTrajectoryControl(false)) {
    stream.println("Trajectory control has been disabled.");
  }
  if (EnableWheelControl(false)) {
    stream.println("Wheel speed control has been disabled.");
  }
}

void WriteMotorsPWMCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Sets the PWM duty cycle of the left and right motors of the robot between 0 (stopped) and 1 (full speed).");
}

void WriteMotorsAngularSpeedCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  if (command_line.num_params != 2) {
    stream.println("This command takes two floating point arguments: write motors angular_speed left_radians_per_second right_radians_per_second.");
    return;
  }
  const StatusOr<float> left_speed = command_line.params[0].ToFloat();
  if (!left_speed.ok()) {
    stream.println("Unable to parse left motor's angular speed.");
    return;
  }
  const StatusOr<float> right_speed = command_line.params[1].ToFloat();
  if (!right_speed.ok()) {
    stream.println("Unable to parse right motor's angular speed.");
    return;
  }

  left_wheel.SetAngularSpeed(*left_speed);
  right_wheel.SetAngularSpeed(*right_speed);

  if (EnableTrajectoryControl(false)) {
    stream.println("Trajectory control has been disabled.");
  }
  if (!EnableTrajectoryControl(true)) {
    stream.println("Wheel speed control had been enabled.");
  }
}

void WriteMotorsAngularSpeedCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Sets the angular speed of the left and right motors of the robot in rad/s.");
}

void WriteMotorsLinearSpeedCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  if (command_line.num_params != 2) {
    stream.println("This command takes two floating point arguments: write motors linear_speed left_meters_per_second right_meters_per_second.");
    return;
  }
  const StatusOr<float> left_speed = command_line.params[0].ToFloat();
  if (!left_speed.ok()) {
    stream.println("Unable to parse left motor's linear speed.");
    return;
  }
  const StatusOr<float> right_speed = command_line.params[1].ToFloat();
  if (!right_speed.ok()) {
    stream.println("Unable to parse right motor's linear speed.");
    return;
  }
  
  left_wheel.SetLinearSpeed(*left_speed);
  right_wheel.SetLinearSpeed(*right_speed);

  if (EnableTrajectoryControl(false)) {
    stream.println("Trajectory control has been disabled.");
  }
  if (!EnableWheelControl(true)) {
    stream.println("Wheel speed control had been enabled.");
  }
}

void WriteMotorsLinearSpeedCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Sets the linear speed of the left and right motors of the robot in m/s.");
}

void WriteServosCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  if (command_line.num_params < 2) {
    stream.println("Two parameters are required: pitch and roll, in radians. Use 'help write servos' for details.");
    return;
  }

  char number_str[max(command_line.params[0].Length(), command_line.params[1].Length()) + 1];
  command_line.params[0].ToCString(number_str);
  float pitch_radians;
  if (sscanf(number_str, "%f", &pitch_radians) != 1) {
    stream.printf("Wrong pitch radians '%s'.\n", number_str);
    return;
  }
  command_line.params[1].ToCString(number_str);
  float roll_radians;
  if (sscanf(number_str, "%f", &roll_radians) != 1) {
    stream.printf("Wrong roll radians '%s'.\n", number_str);
    return;
  }

  SetHeadPitchDegrees(pitch_radians);
  SetHeadRollDegrees(roll_radians);
}

void WriteServosCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Sets the angles of the head servos: write servos pitch_radians roll_radians.");
}

void CheckMCUCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  CheckMCU(stream);
}

void CheckMCUCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Checks the MCU type.");
}

void CheckSRAMCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  if (!CheckMCU(null_stream)) {
    stream.println("Cannot check SRAM on an unsupported MCU.");
    return;
  }
  CheckSRAM(stream);
}

void CheckSRAMCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Checks the integrity of the SRAM memory in the MCU.");
}

void CheckEEPROMCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  CheckEEPROM(stream);
}

void CheckEEPROMCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Checks the integrity of the EEPROM memory.");
}

void CheckBatteryCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  CheckBattery(stream);
}

void CheckBatteryCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Checks the battery has an appropriate voltage.");
}

void CheckTimerCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  CheckTimer(stream);
}

void CheckTimerCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Checks the MCU's internal timer.");
}

void CheckMotorsCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  CheckMotors(stream, /*check_preconditions=*/true);
}

void CheckMotorsCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Rotates the wheels backward and forkward for you to check if the motors work.");
}

void CheckEncodersCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  CheckEncoders(stream, /*check_preconditions=*/true);
}

void CheckEncodersCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Waits for you to rotate the wheels to check encoder signals.");
}

void CheckBodyIMUCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  CheckBodyIMU(stream, /*check_preconditions*/true);
}

void CheckBodyIMUCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Waits for you to move the robot as instructed to check the IMU output.");
}

void CheckBodyMotionCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  CheckBodyMotion(stream, /*check_preconditions*/true);
}

void CheckBodyMotionCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Checks the base motion system by activating the motors and monitoring the readings from the wheel encoders and the body IMU.");
}

void CalibrateBodyIMUCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  body_imu.StartCalibration();
  TimerSecondsType last_print_seconds_ = GetTimerSeconds();
  while(!stream.available()) {
    const auto status = body_imu.GetCalibrationStatus();
    const auto now = GetTimerSeconds();
    if (now - last_print_seconds_ >= 0.5) {
      PrintBodyIMUCalibrationStatus(stream, status);
      last_print_seconds_ = now;
    }
    if (status.IsFullyCalibrated()) {
      break;
    }
  }
  while(stream.available()) {
    stream.read();    
  }

  if (!body_imu.IsCalibrated()) {
    body_imu.StopCalibration();
    stream.println("ERROR: Calibration interrupted by user.");
    return;
  }
  
  body_imu.StopCalibration();
  PrintBodyIMUCalibrationStatus(stream, body_imu.GetCalibrationStatus());

  stream.print("Saving calibration data to EEPROM");
  if (body_imu.SaveCalibrationData()) { stream.println(": OK."); }
  else { stream.println(": ERROR."); }

  stream.println("OK.");
}

void CalibrateBodyIMUCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Runs the calibration routine of the body IMU.");
}

void ResetMCUCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  if (!CheckMCU(null_stream)) {
    stream.println("Cannot reset an unsupported MCU.");
    return;
  }
  stream.println("Resetting the MCU, including its peripherals...");
  // Wait for command and progress message to be printed to the console.
  SleepForSeconds(0.25);
  // Write the reset request to the Application Interrupt and Reset Control Register (AIRCR).
  // VECTKEY [bits 31:16]: Vector key bits. On writes, write 0x05FA to VECTKEY, otherwise the write is ignored.
  // System reset request bit [bit 2]: 1 = Request a system reset.
  SCB_AIRCR = 0x05FA0004;
}

void ResetMCUCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Resets the MCU, including the peripherals.");
}

void Console::ProcessCommandLine() {
  input_stream_.printf("\n> %s\n", command_line_);

  CommandLine cline;
  const char *cline_ptr = command_line_;

  // Find command name.
  cline_ptr = SkipSpaces(cline_ptr);  
  cline.command_name.start = cline_ptr;
  cline_ptr = SkipNonSpaces(cline_ptr);
  cline.command_name.end = cline_ptr;
  if (cline.command_name.Length() == 0) { return; }

  // Find parameters.
  cline.num_params = 0;
  while (cline.num_params < CONSOLE_MAX_NUM_COMMAND_PARAMS && *cline_ptr != '\0') {
    cline_ptr = SkipSpaces(cline_ptr);
    if (*cline_ptr == '\0') { break; }
    cline.params[cline.num_params].start = cline_ptr;
    cline_ptr = SkipNonSpaces(cline_ptr);
    cline.params[cline.num_params].end = cline_ptr;
    ++cline.num_params;
  }

  CommandHandler *const command_handler = interpreter_.FindCommandHandler(cline.command_name);
  if (command_handler != nullptr) {
    command_handler->Run(input_stream_, cline);
  } else {
    input_stream_.println("Command not found.");
  }
}

bool Console::SchedulePeriodicCommand(Stream *stream, const CommandLine &command_line, TimerNanosType period_ns) {
  CommandHandler *command_handler = interpreter_.FindCommandHandler(command_line.command_name);
  if (command_handler == nullptr) { return false; }
  periodic_command_.set(stream, command_handler, command_line, period_ns);  
  return true;
}

void EveryCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  if (command_line.num_params < 2) {
    stream.println("At least two parameters are required after 'every': period and command. Run 'help every' for details.");
    return;
  }
  StatusOr<TimerNanosType> period_ns = NanosFromDurationString(command_line.params[0]);
  if (!period_ns.ok()) {
    stream.print("Invalid command period '");
    command_line.params[0].Print(stream);
    stream.println("'.");
    return;
  }
  const CommandLine periodic_command_line = command_line.ShiftLeft().ShiftLeft();
  if (!console_.SchedulePeriodicCommand(&stream, periodic_command_line, *period_ns)) {
    stream.print("Wrong command '");
    periodic_command_line.command_name.Print(stream);
    stream.println("'.");
  }
}

void EveryCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("When used before another command, 'every' runs the command repeatedly with the given period.");
}

void EveryCommandHandler::Help(Stream &stream, const CommandLine &command_line) {
  Describe(stream, command_line);
  stream.println("It goes on until ENTER is hit or a new command is sent.");
  stream.println("Format: 'every period command', where period is a floating point number with a units suffix (s, ms, us, ns), e.g. 1.23ms. No suffix means seconds.");
}

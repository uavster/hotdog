#ifndef CONSOLE_INCLUDED_
#define CONSOLE_INCLUDED_

#include <Arduino.h>
#include "logger_interface.h"
#include "string_view.h"
#include "periodic_runnable.h"

#define CONSOLE_MAX_COMMAND_LINE_LENGTH 128
#define CONSOLE_MAX_NUM_COMMANDS 10
#define CONSOLE_MAX_NUM_COMMAND_PARAMS 10
#define CONSOLE_MAX_COMMAND_NAME_LENGTH 16

struct CommandLine {
  StringView command_name;
  int num_params;
  StringView params[CONSOLE_MAX_NUM_COMMAND_PARAMS];

  CommandLine ShiftLeft() const;
  CommandLine DeepCopy(char *dest_buffer) const;
};

class CommandInterpreter;

class CommandHandler {
public:
  CommandHandler(const char *name) {
    ASSERT(strlen(name) < sizeof(name_));
    strcpy(name_, name);
  }

  virtual void Run(Stream &stream, const CommandLine &command_line) = 0;
  virtual void Describe(Stream &stream, const CommandLine &command_line) = 0;
  virtual void Help(Stream &stream, const CommandLine &command_line) {
    Describe(stream, command_line);
  }

  bool Matches(const StringView &s) const {
    return s == StringView(name_);
  }

  const char *name() const {
    return name_;
  }

  virtual const CommandInterpreter *interpreter() const {
    return nullptr;
  }

private:
  char name_[CONSOLE_MAX_COMMAND_NAME_LENGTH];
};

class PeriodicCommand : public PeriodicRunnable {
public:
  PeriodicCommand()
    : PeriodicRunnable("user_command"), stream_(nullptr), command_handler_(nullptr) {}

  // Does not take ownsership of the pointees, which must outlive this object.
  PeriodicCommand(Stream *stream, CommandHandler *command_handler, const CommandLine &command_line, TimerNanosType period_ns)
    : PeriodicRunnable("user_command") {
      set(stream, command_handler, command_line, period_ns);
  }

  // Does not take ownsership of the pointees, which must outlive this object.
  void set(Stream *stream, CommandHandler *command_handler, const CommandLine &command_line, TimerNanosType period_ns) {
    stream_ = ASSERT_NOT_NULL(stream);
    command_handler_ = ASSERT_NOT_NULL(command_handler);
    command_line_ = command_line.DeepCopy(command_line_buffer_);
    period_nanos(period_ns);
  }

  // The default assignment would be unsafe because it would not translate the 
  // pointers in the command line's string views to the destination object.
  // It is safer to delete it and force the caller to use set() directly on the
  // destination object.
  PeriodicCommand &operator=(const PeriodicCommand &other) = delete;
  PeriodicCommand &operator=(PeriodicCommand &&other) = delete;

protected:
  void RunAfterPeriod(TimerNanosType now_nanos, TimerNanosType nanos_since_last_call) override {
    command_handler_->Run(*stream_, command_line_);
  }

private:
  Stream *stream_;
  CommandHandler *command_handler_;
  char command_line_buffer_[CONSOLE_MAX_COMMAND_LINE_LENGTH];
  CommandLine command_line_;
};

class CommandInterpreter {
public:
  template<int num_command_handlers>
  CommandInterpreter(CommandHandler *const (&&command_handlers)[num_command_handlers])
    : num_command_handlers_(num_command_handlers) {
    static_assert(num_command_handlers <= sizeof(command_handlers_) / sizeof(command_handlers_[0]));
    for (int i = 0; i < num_command_handlers; ++i) { command_handlers_[i] = command_handlers[i]; }
  }

  CommandHandler *const *command_handlers() const {
    return command_handlers_;
  }
  int num_command_handlers() const {
    return num_command_handlers_;
  }

  CommandHandler *FindCommandHandler(const StringView &command_name) const;
  void PrintCommandDescriptions(Stream &stream, const CommandLine &command_line) const;

private:
  int num_command_handlers_;
  CommandHandler *command_handlers_[CONSOLE_MAX_NUM_COMMANDS];
};

class HelpCommandHandler : public CommandHandler {
public:
  HelpCommandHandler(CommandInterpreter *interpreter)
    : CommandHandler("help"), interpreter_(*ASSERT_NOT_NULL(interpreter)) {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;

private:
  CommandInterpreter &interpreter_;
};

class CategoryHandler : public CommandHandler {
public:
  template<int num_command_handlers>
  CategoryHandler(const char *name, CommandHandler *const (&&command_handlers)[num_command_handlers])
    : CommandHandler(name), interpreter_(std::move(command_handlers)) {}

  virtual void Run(Stream &stream, const CommandLine &command_line) override;
  virtual void Help(Stream &stream, const CommandLine &command_line) override;

  const CommandInterpreter *interpreter() const {
    return &interpreter_;
  }

private:
  CommandInterpreter interpreter_;
};

class VersionCommandHandler : public CommandHandler {
public:
  VersionCommandHandler()
    : CommandHandler("version") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class ReadBatteryCommandHandler : public CommandHandler {
public:
  ReadBatteryCommandHandler()
    : CommandHandler("battery") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class ReadBodyIMUOrientationCommandHandler : public CommandHandler {
public:
  ReadBodyIMUOrientationCommandHandler() : CommandHandler("orientation") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;  
  void Help(Stream &stream, const CommandLine &command_line) override;  
};

class ReadBodyIMUAccelerationCommandHandler : public CommandHandler {
public:
  ReadBodyIMUAccelerationCommandHandler() : CommandHandler("acceleration") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;  
  void Help(Stream &stream, const CommandLine &command_line) override;  
};

class ReadBodyIMUCalibrationStatusCommandHandler : public CommandHandler {
public:
  ReadBodyIMUCalibrationStatusCommandHandler() : CommandHandler("status") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;  
};

class ReadBodyIMUCalibrationDataCommandHandler : public CommandHandler {
public:
  ReadBodyIMUCalibrationDataCommandHandler() : CommandHandler("data") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;  
};

class ReadBodyIMUCalibrationCommandHandler : public CategoryHandler {
public:
  ReadBodyIMUCalibrationCommandHandler() 
    : CategoryHandler("calibration", { &read_body_imu_calibration_status_handler_, &read_body_imu_calibration_data_handler_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Reads calibration properties from the body IMU.");
  }

private:
  ReadBodyIMUCalibrationStatusCommandHandler read_body_imu_calibration_status_handler_;
  ReadBodyIMUCalibrationDataCommandHandler read_body_imu_calibration_data_handler_;
};

class ReadBodyIMUCommandHandler : public CategoryHandler {
public:
  ReadBodyIMUCommandHandler() 
    : CategoryHandler("body_imu", { &read_bodyimu_orientation_, &read_bodyimu_acceleration_, &read_bodyimu_calibration_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Reads from the IMU at the robot's body.");    
  }

private:
  ReadBodyIMUOrientationCommandHandler read_bodyimu_orientation_;
  ReadBodyIMUAccelerationCommandHandler read_bodyimu_acceleration_;
  ReadBodyIMUCalibrationCommandHandler read_bodyimu_calibration_;
};

class ReadTimerUnitsCommandHandler : public CommandHandler {
public:
  ReadTimerUnitsCommandHandler(const char *command_name, const char *units_name, uint64_t nanos_to_units_divisor) 
    : CommandHandler(command_name), nanos_to_units_divisor_(nanos_to_units_divisor) {
      ASSERT(strlen(units_name) < sizeof(units_name_));
      strcpy(units_name_, units_name);
    }

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;

private:
  char units_name_[16];
  uint64_t nanos_to_units_divisor_;
};

class ReadTimerTicksCommandHandler : public CommandHandler {
public:
  ReadTimerTicksCommandHandler() : CommandHandler("ticks") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;  
};

class ReadTimerNanosCommandHandler : public ReadTimerUnitsCommandHandler {
public:
  ReadTimerNanosCommandHandler() : ReadTimerUnitsCommandHandler("ns", "nanoseconds", 1) {}
};

class ReadTimerMicrosCommandHandler : public ReadTimerUnitsCommandHandler {
public:
  ReadTimerMicrosCommandHandler() : ReadTimerUnitsCommandHandler("us", "microseconds", 1000) {}
};

class ReadTimerMillisCommandHandler : public ReadTimerUnitsCommandHandler {
public:
  ReadTimerMillisCommandHandler() : ReadTimerUnitsCommandHandler("ms", "milliseconds", 1'000'000) {}
};

class ReadTimerNoUnitsCommandHandler : public ReadTimerUnitsCommandHandler {
public:
  ReadTimerNoUnitsCommandHandler() : ReadTimerUnitsCommandHandler("", "seconds", 1'000'000'000) {}
};

class ReadTimerSecondsCommandHandler : public ReadTimerUnitsCommandHandler {
public:
  ReadTimerSecondsCommandHandler() : ReadTimerUnitsCommandHandler("s", "seconds", 1'000'000'000) {}
};

class ReadTimerCommandHandler : public CategoryHandler {
public:
  ReadTimerCommandHandler() 
    : CategoryHandler("timer", { 
        &read_timer_nounits_handler_, &read_timer_ticks_handler_, &read_timer_nanos_handler_, &read_timer_micros_handler_, 
        &read_timer_millis_handler_, &read_timer_seconds_handler_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Reads the local monotonic timer started at boot time.");
  }

private:
  ReadTimerNoUnitsCommandHandler read_timer_nounits_handler_;
  ReadTimerTicksCommandHandler read_timer_ticks_handler_;
  ReadTimerNanosCommandHandler read_timer_nanos_handler_;
  ReadTimerMicrosCommandHandler read_timer_micros_handler_;
  ReadTimerMillisCommandHandler read_timer_millis_handler_;
  ReadTimerSecondsCommandHandler read_timer_seconds_handler_;  
};

class ReadGlobalTimerUnitsCommandHandler : public CommandHandler {
public:
  ReadGlobalTimerUnitsCommandHandler(const char *command_name, const char *units_name, uint64_t nanos_to_units_divisor) 
    : CommandHandler(command_name), nanos_to_units_divisor_(nanos_to_units_divisor) {
      ASSERT(strlen(units_name) < sizeof(units_name_));
      strcpy(units_name_, units_name);
    }

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;

private:
  char units_name_[16];
  uint64_t nanos_to_units_divisor_;
};

class ReadGlobalTimerNanosCommandHandler : public ReadGlobalTimerUnitsCommandHandler {
public:
  ReadGlobalTimerNanosCommandHandler() : ReadGlobalTimerUnitsCommandHandler("ns", "nanoseconds", 1) {}
};

class ReadGlobalTimerMicrosCommandHandler : public ReadGlobalTimerUnitsCommandHandler {
public:
  ReadGlobalTimerMicrosCommandHandler() : ReadGlobalTimerUnitsCommandHandler("us", "microseconds", 1000) {}
};

class ReadGlobalTimerMillisCommandHandler : public ReadGlobalTimerUnitsCommandHandler {
public:
  ReadGlobalTimerMillisCommandHandler() : ReadGlobalTimerUnitsCommandHandler("ms", "milliseconds", 1'000'000) {}
};

class ReadGlobalTimerNoUnitsCommandHandler : public ReadGlobalTimerUnitsCommandHandler {
public:
  ReadGlobalTimerNoUnitsCommandHandler() : ReadGlobalTimerUnitsCommandHandler("", "seconds", 1'000'000'000) {}
};

class ReadGlobalTimerSecondsCommandHandler : public ReadGlobalTimerUnitsCommandHandler {
public:
  ReadGlobalTimerSecondsCommandHandler() : ReadGlobalTimerUnitsCommandHandler("s", "seconds", 1'000'000'000) {}
};

class ReadGlobalTimerCommandHandler : public CategoryHandler {
public:
  ReadGlobalTimerCommandHandler() 
    : CategoryHandler("global_timer", { 
        &read_timer_nounits_handler_, &read_timer_nanos_handler_, &read_timer_micros_handler_, 
        &read_timer_millis_handler_, &read_timer_seconds_handler_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Reads the global monotonic timer started at boot time.");
  }

private:
  ReadGlobalTimerNoUnitsCommandHandler read_timer_nounits_handler_;
  ReadGlobalTimerNanosCommandHandler read_timer_nanos_handler_;
  ReadGlobalTimerMicrosCommandHandler read_timer_micros_handler_;
  ReadGlobalTimerMillisCommandHandler read_timer_millis_handler_;
  ReadGlobalTimerSecondsCommandHandler read_timer_seconds_handler_;  
};

class ReadEncodersTicksCommandHandler : public CommandHandler {
public:
  ReadEncodersTicksCommandHandler() : CommandHandler("ticks") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class ReadEncodersAngularSpeedCommandHandler : public CommandHandler {
public:
  ReadEncodersAngularSpeedCommandHandler() : CommandHandler("angular_speed") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class ReadEncodersLinearSpeedCommandHandler : public CommandHandler {
public:
  ReadEncodersLinearSpeedCommandHandler() : CommandHandler("linear_speed") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class ReadEncodersCommandHandler : public CategoryHandler {
public:
  ReadEncodersCommandHandler() 
    : CategoryHandler("encoders", { &read_encoders_ticks_handler_, &read_encoders_angular_speed_handler_, &read_encoders_linear_speed_handler_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Reads measurements from the encoders.");
  }

private:
  ReadEncodersTicksCommandHandler read_encoders_ticks_handler_;
  ReadEncodersAngularSpeedCommandHandler read_encoders_angular_speed_handler_;
  ReadEncodersLinearSpeedCommandHandler read_encoders_linear_speed_handler_;
};

class ReadCommandHandler : public CategoryHandler {
public:
  ReadCommandHandler()
    : CategoryHandler("read", { &read_timer_handler_, &read_global_timer_handler_, &read_battery_handler_, &read_bodyimu_handler_, &read_encoders_handler_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Reads from an information source on the robot.");    
  }

private:
  ReadTimerCommandHandler read_timer_handler_;
  ReadGlobalTimerCommandHandler read_global_timer_handler_;
  ReadBatteryCommandHandler read_battery_handler_;
  ReadBodyIMUCommandHandler read_bodyimu_handler_;
  ReadEncodersCommandHandler read_encoders_handler_;
};

class WriteMotorsPWMCommandHandler : public CommandHandler {
public:
  WriteMotorsPWMCommandHandler() : CommandHandler("pwm") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class WriteMotorsAngularSpeedCommandHandler : public CommandHandler {
public:
  WriteMotorsAngularSpeedCommandHandler() : CommandHandler("angular_speed") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class WriteMotorsLinearSpeedCommandHandler : public CommandHandler {
public:
  WriteMotorsLinearSpeedCommandHandler() : CommandHandler("linear_speed") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class WriteMotorsCommandHandler : public CategoryHandler {
public:
  WriteMotorsCommandHandler()
    : CategoryHandler("motors", { &write_motors_pwm_handler_, &write_motors_angular_speed_handler_, &write_motors_linear_speed_handler_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Writes open-loop commands to the robot motors.");
  }
  
private:
  WriteMotorsPWMCommandHandler write_motors_pwm_handler_;
  WriteMotorsAngularSpeedCommandHandler write_motors_angular_speed_handler_;
  WriteMotorsLinearSpeedCommandHandler write_motors_linear_speed_handler_;
};

class WriteWheelsAngularSpeedCommandHandler : public CommandHandler {
public:
  WriteWheelsAngularSpeedCommandHandler() : CommandHandler("angular_speed") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class WriteWheelsLinearSpeedCommandHandler : public CommandHandler {
public:
  WriteWheelsLinearSpeedCommandHandler() : CommandHandler("linear_speed") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class WriteWheelsCommandHandler : public CategoryHandler {
public:
  WriteWheelsCommandHandler()
    : CategoryHandler("wheels", { &write_wheels_angular_speed_handler_, &write_wheels_linear_speed_handler_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Sends commands to the closed-loop wheel speed controller.");
  }
  
private:
  WriteWheelsAngularSpeedCommandHandler write_wheels_angular_speed_handler_;
  WriteWheelsLinearSpeedCommandHandler write_wheels_linear_speed_handler_;
};

class WriteServosCommandHandler : public CommandHandler {
public:
  WriteServosCommandHandler() : CommandHandler("servos") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class WriteCommandHandler : public CategoryHandler {
public:
  WriteCommandHandler()
    : CategoryHandler("write", { &write_motors_handler_, &write_wheels_handler_, &write_servos_handler_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Writes to an information sink on the robot.");
  }

private:
  WriteMotorsCommandHandler write_motors_handler_;
  WriteWheelsCommandHandler write_wheels_handler_;
  WriteServosCommandHandler write_servos_handler_;
};

class Console;

class EveryCommandHandler : public CommandHandler {
public:
  EveryCommandHandler(Console *console)
    : CommandHandler("every"), console_(*ASSERT_NOT_NULL(console)) {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
  void Help(Stream &stream, const CommandLine &command_line) override;

private:
  Console &console_;
};

class CheckMCUCommandHandler : public CommandHandler {
public:
  CheckMCUCommandHandler() : CommandHandler("mcu") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class CheckSRAMCommandHandler : public CommandHandler {
public:
  CheckSRAMCommandHandler() : CommandHandler("sram") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class CheckEEPROMCommandHandler : public CommandHandler {
public:
  CheckEEPROMCommandHandler() : CommandHandler("eeprom") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class CheckBatteryCommandHandler : public CommandHandler {
public:
  CheckBatteryCommandHandler() : CommandHandler("battery") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class CheckTimerCommandHandler : public CommandHandler {
public:
  CheckTimerCommandHandler() : CommandHandler("timer") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class CheckMotorsCommandHandler : public CommandHandler {
public:
  CheckMotorsCommandHandler() : CommandHandler("motors") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class CheckEncodersCommandHandler : public CommandHandler {
public:
  CheckEncodersCommandHandler() : CommandHandler("encoders") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class CheckBodyIMUCommandHandler : public CommandHandler {
public:
  CheckBodyIMUCommandHandler() : CommandHandler("body_imu") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class CheckBodyMotionCommandHandler : public CommandHandler {
public:
  CheckBodyMotionCommandHandler() : CommandHandler("body_motion") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class CheckCommandHandler : public CategoryHandler {
public:
  CheckCommandHandler()
    : CategoryHandler("check", { 
      &check_mcu_handler_, &check_sram_handler_, &check_eeprom_handler_, &check_timer_handler_, &check_battery_handler_, 
      &check_motors_handler_, &check_encoders_handler_, &check_body_imu_command_handler_,
      &check_body_motion_command_handler_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Checks different subsystems on the robot.");    
  }

private:
  CheckMCUCommandHandler check_mcu_handler_;
  CheckSRAMCommandHandler check_sram_handler_;
  CheckEEPROMCommandHandler check_eeprom_handler_;
  CheckTimerCommandHandler check_timer_handler_;
  CheckBatteryCommandHandler check_battery_handler_;
  CheckMotorsCommandHandler check_motors_handler_;
  CheckEncodersCommandHandler check_encoders_handler_;
  CheckBodyIMUCommandHandler check_body_imu_command_handler_;
  CheckBodyMotionCommandHandler check_body_motion_command_handler_;
};

class CalibrateBodyIMUCommandHandler : public CommandHandler {
public:
  CalibrateBodyIMUCommandHandler() : CommandHandler("body_imu") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class CalibrateCommandHandler : public CategoryHandler {
public:
  CalibrateCommandHandler() : CategoryHandler("calibrate", { &calibrate_body_imu_handler_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Calibrates different subsystems on the robot.");    
  }

private:
  CalibrateBodyIMUCommandHandler calibrate_body_imu_handler_;
};

class ResetMCUCommandHandler : public CommandHandler {
public:
  ResetMCUCommandHandler() : CommandHandler("mcu") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class ResetCommandHandler : public CategoryHandler {
public:
  ResetCommandHandler() : CategoryHandler("reset", { &reset_mcu_handler_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Resets a subsystem.");    
  }

private:
  ResetMCUCommandHandler reset_mcu_handler_;
};

class Console {
public:
  Console(Stream *input_stream)
    : input_stream_(*ASSERT_NOT_NULL(input_stream)),
      command_line_length_(0),
      help_command_handler_(&interpreter_),
      every_command_handler_(this),
      interpreter_({ &help_command_handler_,
                     &version_command_handler_,
                     &read_command_handler_,
                     &write_command_handler_,
                     &check_command_handler_,
                     &calibrate_command_handler_,
                     &reset_command_handler_,
                     &every_command_handler_ }) {
  }
  void Run();

public:
  bool SchedulePeriodicCommand(Stream *stream, const CommandLine &command_line, TimerNanosType period_ns);

private:
  void ProcessCommandLine();

  Stream &input_stream_;
  uint8_t command_line_length_;
  char command_line_[CONSOLE_MAX_COMMAND_LINE_LENGTH];

  HelpCommandHandler help_command_handler_;
  VersionCommandHandler version_command_handler_;
  ReadCommandHandler read_command_handler_;
  WriteCommandHandler write_command_handler_;
  CheckCommandHandler check_command_handler_;
  CalibrateCommandHandler calibrate_command_handler_;
  ResetCommandHandler reset_command_handler_;
  EveryCommandHandler every_command_handler_;

  CommandInterpreter interpreter_;
  PeriodicCommand periodic_command_;
};

#endif  // CONSOLE_INCLUDED_
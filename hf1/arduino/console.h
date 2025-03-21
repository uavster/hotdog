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
  CommandInterpreter(CommandHandler *const (&command_handlers)[num_command_handlers])
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
  CategoryHandler(const char *name, CommandHandler *const (&command_handlers)[num_command_handlers])
    : CommandHandler(name), interpreter_((command_handlers)) {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Help(Stream &stream, const CommandLine &command_line) override;

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
  ReadBodyIMUOrientationCommandHandler() 
    : CommandHandler("orientation") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;  
};

class ReadBodyIMUCommandHandler : public CategoryHandler {
public:
  ReadBodyIMUCommandHandler() 
    : CategoryHandler("bodyimu", { &read_bodyimu_orientation_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Reads from the IMU at the robot's body.");    
  }

private:
  ReadBodyIMUOrientationCommandHandler read_bodyimu_orientation_;
};

class ReadCommandHandler : public CategoryHandler {
public:
  ReadCommandHandler()
    : CategoryHandler("read", { &read_battery_handler_, &read_bodyimu_handler_ }) {}

  void Describe(Stream &stream, const CommandLine &command_line) override {
    stream.println("Reads from different information sources on the robot.");    
  }

private:
  ReadBatteryCommandHandler read_battery_handler_;
  ReadBodyIMUCommandHandler read_bodyimu_handler_;
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
  EveryCommandHandler every_command_handler_;

  CommandInterpreter interpreter_;
  PeriodicCommand periodic_command_;
};

#endif  // CONSOLE_INCLUDED_
#ifndef CONSOLE_INCLUDED_
#define CONSOLE_INCLUDED_

#include <Arduino.h>
#include "logger_interface.h"
#include "string_view.h"

#define CONSOLE_MAX_COMMAND_LINE_LENGTH 128
#define CONSOLE_MAX_NUM_COMMANDS 10
#define CONSOLE_MAX_NUM_COMMAND_PARAMS 10
#define CONSOLE_MAX_COMMAND_NAME_LENGTH 16

struct CommandLine {
  StringView command_name;
  int num_params;
  StringView params[CONSOLE_MAX_NUM_COMMAND_PARAMS];

  CommandLine ShiftLeft() const {
    CommandLine new_line;
    new_line.command_name = params[0];
    new_line.num_params = num_params - 1;
    for (int i = 0; i < new_line.num_params; ++i) {
      new_line.params[i] = params[i + 1];
    }
    return new_line;
  }
};

class CommandInterpreter;

class CommandHandler {  
public:
  CommandHandler(const char *name) { strcpy(name_, name); }

  virtual void Run(Stream &stream, const CommandLine &command_line) = 0;
  virtual void Describe(Stream &stream, const CommandLine &command_line) = 0;
  virtual void Help(Stream &stream, const CommandLine &command_line) { Describe(stream, command_line); }

  bool Matches(const StringView &s) const {
    return s == StringView(name_);
  }

  const char *name() const { return name_; }

  virtual const CommandInterpreter *interpreter() const { return nullptr; }

private:
  char name_[CONSOLE_MAX_COMMAND_NAME_LENGTH];
};

class CommandInterpreter {
public:
  template<int num_command_handlers>
  CommandInterpreter(CommandHandler *const (&&command_handlers)[num_command_handlers]) 
    : num_command_handlers_(num_command_handlers) {
      static_assert(num_command_handlers <= sizeof(command_handlers_) / sizeof(command_handlers_[0]));
      for (int i = 0; i < num_command_handlers; ++i) { command_handlers_[i] = command_handlers[i]; }
    }

  CommandHandler *const *command_handlers() const { return command_handlers_; }
  int num_command_handlers() const { return num_command_handlers_; }

  CommandHandler *FindCommandHandler(const StringView &command_name) const;
  void PrintCommandDescriptions(Stream &stream, const CommandLine &command_line) const;

private:
  int num_command_handlers_;
  CommandHandler *command_handlers_[CONSOLE_MAX_NUM_COMMANDS];
};

class HelpCommandHandler : public CommandHandler {
public:
  HelpCommandHandler(CommandInterpreter *interpreter) : CommandHandler("help"), interpreter_(*ASSERT_NOT_NULL(interpreter)) {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;

private:
  CommandInterpreter &interpreter_;
};

class VersionCommandHandler : public CommandHandler {
public:
  VersionCommandHandler() : CommandHandler("version") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class ReadBatteryCommandHandler : public CommandHandler {
public:
  ReadBatteryCommandHandler() : CommandHandler("battery") {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
};

class ReadCommandHandler : public CommandHandler {
public:
  ReadCommandHandler() : CommandHandler("read"), interpreter_({ &read_battery_handler_ }) {}

  void Run(Stream &stream, const CommandLine &command_line) override;
  void Describe(Stream &stream, const CommandLine &command_line) override;
  void Help(Stream &stream, const CommandLine &command_line) override;

  const CommandInterpreter *interpreter() const override { return &interpreter_; }

private:
  ReadBatteryCommandHandler read_battery_handler_;
  CommandInterpreter interpreter_;
};

class Console {
public:
  Console(Stream *input_stream) 
    : input_stream_(*ASSERT_NOT_NULL(input_stream)), 
      command_line_length_(0), 
      help_command_handler_(&interpreter_),
      interpreter_({
        &help_command_handler_, 
        &version_command_handler_,
        &read_command_handler_
      }) {
  }
  void Run();

private:
  void ProcessCommandLine();
  
  Stream &input_stream_;
  uint8_t command_line_length_;
  char command_line_[CONSOLE_MAX_COMMAND_LINE_LENGTH];

  HelpCommandHandler help_command_handler_;
  VersionCommandHandler version_command_handler_;
  ReadCommandHandler read_command_handler_;

  CommandInterpreter interpreter_;
};

#endif  // CONSOLE_INCLUDED_
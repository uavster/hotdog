#include "console.h"
#include "battery.h"

namespace {
const char *SkipSpaces(const char *str) {  
  while (*str == ' ' && *str != '\0') { ++str; }
  return str;
}

const char *SkipNonSpaces(const char *str) {
  while (*str != ' ' && *str != '\0') { ++str; }
  return str;
}

}

void Console::Run() {
  while (input_stream_.available()) {
    const int c = input_stream_.read();
    if (c < 0) { return; }
    if (c == '\n' || c == '\r' || c == '\0' || command_line_length_ >= sizeof(command_line_) - 1) {
      command_line_[command_line_length_] = '\0';
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

void ReadCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  CommandHandler *handler = nullptr;
  if (command_line.num_params < 1) {
    stream.print("Please specify what to read: ");
  } else {
    handler = interpreter_.FindCommandHandler(command_line.params[0]);
    if (handler == nullptr) {
      stream.printf("Wrong argument '%s'. Please use one of these: ", command_line.params[0]);
    }
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

void ReadCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Reads from different information sources on the robot.");
}

void ReadCommandHandler::Help(Stream &stream, const CommandLine &command_line) {
  Describe(stream, command_line);
  stream.println("Format: 'read source', where source is one of:");
  interpreter_.PrintCommandDescriptions(stream, command_line);
}

void ReadBatteryCommandHandler::Run(Stream &stream, const CommandLine &command_line) {
  stream.printf("%.1fV\n", GetBatteryVoltage());
}

void ReadBatteryCommandHandler::Describe(Stream &stream, const CommandLine &command_line) {
  stream.println("Prints the battery voltage.");
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
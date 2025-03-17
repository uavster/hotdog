#include "console.h"

#define MAX_NUM_PARAMS 10
#define MAX_COMMAND_NAME_LENGTH 16

struct StringView {
  const char *start;
  const char *end;

  StringView() : start(nullptr), end(nullptr) {}

  StringView(const char *s) : start(s) {
    while(*s != '\0') { ++s; }
    end = s;
  }

  bool Length() const { return end - start; }

  bool operator==(const StringView other) const {
    const char *p1 = start;
    const char *p2 = other.start;
    while(*p1 == *p2 && p1 != end && p2 != other.end) {
      ++p1;
      ++p2;    
    }
    return p1 == end && p2 == other.end;
  }

  void print(Stream &s) const {
    const char *p = start;
    while(p != end) {
      s.write(*p);
      ++p;
    }
  }

  void println(Stream &s) const {
    print(s);
    s.println();
  }  
};

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

struct CommandLine {
  StringView command_name;
  int num_params;
  StringView params[MAX_NUM_PARAMS];
};

class CommandHandler {  
public:
  CommandHandler(const char *name) { strcpy(name_, name); }

  virtual void Run(Stream &stream, const CommandLine &command_line) = 0;
  virtual void Help(Stream &stream, const CommandLine &command_line) = 0;

  bool Matches(const StringView &s) const {
    return s == StringView(name_);
  }

  const char *name() const { return name_; }

private:
  char name_[MAX_COMMAND_NAME_LENGTH];
};

CommandHandler **command_handlers_ptr;

class HelpCommandHandler : public CommandHandler {
public:
  HelpCommandHandler() : CommandHandler("help") {}

  void Run(Stream &stream, const CommandLine &command_line) override {
    if (command_line.num_params == 0) {
      stream.println("Available commands:");
      int i = 0;
      while(command_handlers_ptr[i] != nullptr) {
        stream.printf("%s - ", command_handlers_ptr[i]->name());
        command_handlers_ptr[i]->Help(stream, command_line);
        ++i;
      }
    } else {
      // First parameter is the command name the user needs help about.
      int i = 0;
      while(command_handlers_ptr[i] != nullptr) {
        if (command_line.params[0] == command_handlers_ptr[i]->name()) {
          command_handlers_ptr[i]->Help(stream, command_line);
          break;
        }
        ++i;
      }

    }
  }
  void Help(Stream &stream, const CommandLine &command_line) override {
    stream.println("Prints usage information about available commands.");
  }
};

class VersionCommandHandler : public CommandHandler {
public:
  VersionCommandHandler() : CommandHandler("version") {}

  void Run(Stream &stream, const CommandLine &command_line) override {
    stream.println("HF1 v0.1");
    stream.println("(c) Ignacio Mellado Bataller");
  }
  void Help(Stream &stream, const CommandLine &command_line) override {
    stream.println("Prints the software version.");
  }
};

HelpCommandHandler help_command_handler;
VersionCommandHandler version_command_handler;

CommandHandler *command_handlers[] = {
  &help_command_handler, 
  &version_command_handler,
  nullptr
};

void Console::ProcessCommandLine() {
  command_handlers_ptr = command_handlers;
  input_stream_.printf("%s\n", command_line_);

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
  while (cline.num_params < MAX_NUM_PARAMS && *cline_ptr != '\0') {
    cline_ptr = SkipSpaces(cline_ptr);  
    cline.params[cline.num_params].start = cline_ptr;
    cline_ptr = SkipNonSpaces(cline_ptr);
    cline.params[cline.num_params].end = cline_ptr;
    ++cline.num_params;
  }

  bool command_found = false;
  for (size_t i = 0; i < sizeof(command_handlers) / sizeof(command_handlers[0]); ++i) {
    if (command_handlers[i]->Matches(cline.command_name)) {
      command_handlers[i]->Run(input_stream_, cline);
      command_found = true;
      break;
    }
  }
  if (!command_found) {
    input_stream_.println("Command not found.");
  }
}
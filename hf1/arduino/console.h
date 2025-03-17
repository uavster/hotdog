#ifndef CONSOLE_INCLUDED_
#define CONSOLE_INCLUDED_

#include <Arduino.h>
#include "logger_interface.h"

#define CONSOLE_MAX_COMMAND_LINE_LENGTH 128

class Console {
public:
  Console(Stream *input_stream) : input_stream_(*ASSERT_NOT_NULL(input_stream)), command_line_length_(0) {}
  void Run();

private:
  void ProcessCommandLine();
  
  Stream &input_stream_;
  uint8_t command_line_length_;
  char command_line_[CONSOLE_MAX_COMMAND_LINE_LENGTH];
};

#endif  // CONSOLE_INCLUDED_
#include "logger_interface.h"
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdio.h>
#endif

DefaultLogger default_logger;
LoggerInterface *logger_ = &default_logger;

LoggerInterface *SetLogger(LoggerInterface *new_logger) {
  LoggerInterface *old_logger = logger_;
  logger_ = new_logger;
  return old_logger;
}

void DefaultLogger::Fatal(const char *expr, const char *file_name, int line, const char *msg) {
#ifdef ARDUINO
  Serial.printf("%s is false at line %d of file %s", expr, line, file_name);
  if (msg != nullptr) {
    Serial.printf(":\n%s\n", msg);
  }
  for (;;) {}
#else
  printf("%s is false at line %d of file %s: %s\n", expr, line, file_name, msg);
  if (msg != nullptr) {
    printf(":\n%s\n", msg);
  }
  exit(1);
#endif    
}



#ifndef LOGGER_INTERFACE_INCLUDED__
#define LOGGER_INTERFACE_INCLUDED__

#include "logger_interface.h"
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstdlib>
#include <stdio.h>
#endif

DefaultLogger default_logger;
LoggerInterface *logger_ = &default_logger;

LoggerInterface *SetLogger(LoggerInterface *new_logger) {
  LoggerInterface *old_logger = logger_;
  logger_ = new_logger;
  return old_logger;
}

LoggerInterface *GetLogger() {
  return logger_;
}

void DefaultLogger::Fatal(const char *expr, const char *file_name, int line, const char *msg) {
#ifdef ARDUINO
  Serial.printf("%s is false at line %d of file %s", expr, line, file_name);
  if (msg != nullptr) {
    Serial.printf(":\n%s\n", msg);
  }
  for (;;) {}
#else
  printf("%s is false at line %d of file %s", expr, line, file_name);
  if (msg != nullptr) {
    printf(":\n%s\n", msg);
  }
  exit(1);
#endif    
}

void DefaultLogger::Message(const char *level, const char *file_name, int line, const char *msg) {
#ifdef ARDUINO
  Serial.printf("[%s] %s:%d: %s\n", level, file_name, line, msg);
#else
  printf("[%s] %s:%d: %s\n", level, file_name, line, msg);
#endif    
}

void DefaultLogger::Info(const char *file_name, int line, const char *msg) {
  const static char level[] = "INFO";
  Message(level, file_name, line, msg);
}

void DefaultLogger::Warning(const char *file_name, int line, const char *msg) {
  const static char level[] = "WARNING";
  Message(level, file_name, line, msg);
}

void DefaultLogger::Error(const char *file_name, int line, const char *msg) {
  const static char level[] = "ERROR";
  Message(level, file_name, line, msg);
}

#endif  // LOGGER_INTERFACE_INCLUDED__
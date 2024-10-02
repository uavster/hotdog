#include "logger.h"
#include "led.h"

void Logger::Fatal(const char *expr, const char *file_name, int line, const char *msg) {
  LedShowAssert();
  if (base_logger_ != nullptr) {
    base_logger_->Fatal(expr, file_name, line, msg);
  }
}

void Logger::Info(const char *file_name, int line, const char *msg) {
  if (base_logger_ != nullptr) {
    base_logger_->Info(file_name, line, msg);
  }
}

void Logger::Warning(const char *file_name, int line, const char *msg) {
  if (base_logger_ != nullptr) {
    base_logger_->Warning(file_name, line, msg);
  }
}

void Logger::Error(const char *file_name, int line, const char *msg) {
  if (base_logger_ != nullptr) {
    base_logger_->Error(file_name, line, msg);
  }
}

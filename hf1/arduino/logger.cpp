#include "logger.h"
#include "led.h"

void Logger::Fatal(const char *expr, const char *file_name, int line, const char *msg) {
  LedShowAssert();
  if (base_logger_ != nullptr) {
    base_logger_->Fatal(expr, file_name, line, msg);
  }
}

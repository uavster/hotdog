#include "logger.h"
#include "led.h"
#include "timer.h"

LedRGB *logger_led_rgb = nullptr;

void Logger::LedAssertTimerIsr() {
  if (logger_led_rgb == nullptr) {
    return;
  }
  if (DidTimerCountReachZero()) {
    if (logger_led_rgb->GetColorRGB().red() != 0) {
      logger_led_rgb->SetColor(ColorRGB(0.0f, 0.0f, 0.0f));
    } else {
      logger_led_rgb->SetColor(ColorRGB(1.0f, 0.0f, 0.0f));
    }
  }
}

void Logger::ShowAssertOnLed() {
  NO_TIMER_IRQ {
    logger_led_rgb = led_rgb_;
  }  
  AddTimerIsrWithoutDuplicating(&LedAssertTimerIsr);
}

void Logger::Fatal(const char *expr, const char *file_name, int line, const char *msg) {
  ShowAssertOnLed();
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

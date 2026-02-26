#ifndef LOGGER_INCLUDED_
#define LOGGER_INCLUDED_

#include "logger_interface.h"
#include "led.h"

class Logger : public LoggerInterface {
public:
  Logger() : base_logger_(nullptr), led_rgb_(nullptr) {}
  Logger(LedRGB *led_rgb) : base_logger_(nullptr), led_rgb_(led_rgb) {}

  LoggerInterface **base_logger() { return &base_logger_; }
  const LoggerInterface *base_logger() const { return base_logger_; }

  virtual void Info(const char *file_name, int line, const char *msg);
  virtual void Warning(const char *file_name, int line, const char *msg);
  virtual void Error(const char *file_name, int line, const char *msg);
  virtual void Fatal(const char *expr, const char *file_name, int line, const char *msg = "");

private:
  void ShowAssertOnLed();
  static void LedAssertTimerIsr();

  LoggerInterface *base_logger_;
  LedRGB * const led_rgb_;
};

#endif  // LOGGER_INCLUDED_
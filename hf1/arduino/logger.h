#ifndef LOGGER_INCLUDED_
#define LOGGER_INCLUDED_

#include "logger_interface.h"
#include "led_ui.h"

class Logger : public LoggerInterface {
public:
  Logger() : base_logger_(nullptr), led_ui_(nullptr) {}
  Logger(LedUI *led_ui) : base_logger_(nullptr), led_ui_(led_ui) {}

  LoggerInterface **base_logger() { return &base_logger_; }
  const LoggerInterface *base_logger() const { return base_logger_; }

  virtual void Info(const char *file_name, int line, const char *msg);
  virtual void Warning(const char *file_name, int line, const char *msg);
  virtual void Error(const char *file_name, int line, const char *msg);
  virtual void Fatal(const char *expr, const char *file_name, int line, const char *msg = "");

private:
  LoggerInterface *base_logger_;
  LedUI * const led_ui_;
};

#endif  // LOGGER_INCLUDED_
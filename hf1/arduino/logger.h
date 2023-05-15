#ifndef LOGGER_INCLUDED_
#define LOGGER_INCLUDED_

#include "logger_interface.h"

class Logger : public LoggerInterface {
public:
  Logger() : base_logger_(nullptr) {}

  LoggerInterface **base_logger() { return &base_logger_; }
  const LoggerInterface *base_logger() const { return base_logger_; }

  virtual void Fatal(const char *expr, const char *file_name, int line, const char *msg = "");

private:
  LoggerInterface *base_logger_;
};

#endif  // LOGGER_INCLUDED_
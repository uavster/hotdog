#ifndef LOGGER_INTERFACE_
#define LOGGER_INTERFACE_

class LoggerInterface {
public:
  virtual void Fatal(const char *expr, const char *file_name, int line, const char *msg) = 0;
};

class DefaultLogger : public LoggerInterface {
public:
  virtual void Fatal(const char *expr, const char *file_name, int line, const char *msg);
};

extern LoggerInterface *logger_;

LoggerInterface *SetLogger(LoggerInterface *new_logger);

#define ASSERT(x) if (!(x)) { logger_->Fatal(#x, __FILE__, __LINE__, nullptr); }
#define ASSERTM(x, m) if (!(x)) { logger_->Fatal(#x, __FILE__, __LINE__, m); }

#endif  // LOGGER_INTERFACE_
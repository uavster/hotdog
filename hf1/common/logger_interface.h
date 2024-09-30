#ifndef LOGGER_INTERFACE_
#define LOGGER_INTERFACE_

class LoggerInterface {
public:
  virtual void Info(const char *file_name, int line, const char *msg) = 0;
  virtual void Warning(const char *file_name, int line, const char *msg) = 0;
  virtual void Error(const char *file_name, int line, const char *msg) = 0;
  virtual void Fatal(const char *expr, const char *file_name, int line, const char *msg) = 0;
};

class DefaultLogger : public LoggerInterface {
public:
  virtual void Info(const char *file_name, int line, const char *msg);
  virtual void Warning(const char *file_name, int line, const char *msg);
  virtual void Error(const char *file_name, int line, const char *msg);
  virtual void Fatal(const char *expr, const char *file_name, int line, const char *msg);

protected:
  void Message(const char *level, const char *file_name, int line, const char *msg);
};

LoggerInterface *SetLogger(LoggerInterface *new_logger);
LoggerInterface *GetLogger();

#define LOG_INFO(m) (GetLogger()->Info(__FILE__, __LINE__, m))
#define LOG_WARNING(m) (GetLogger()->Warning(__FILE__, __LINE__, m))
#define LOG_ERROR(m) (GetLogger()->Error(__FILE__, __LINE__, m))

#define ASSERT(x) if (!(x)) { GetLogger()->Fatal(#x, __FILE__, __LINE__, nullptr); }
#define ASSERTM(x, m) if (!(x)) { GetLogger()->Fatal(#x, __FILE__, __LINE__, m); }

#endif  // LOGGER_INTERFACE_
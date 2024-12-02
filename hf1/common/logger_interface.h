#ifndef LOGGER_INTERFACE_
#define LOGGER_INTERFACE_

#include <string.h>

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

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_INFO(m) (GetLogger()->Info(__FILENAME__, __LINE__, m))
#define LOG_WARNING(m) (GetLogger()->Warning(__FILENAME__, __LINE__, m))
#define LOG_ERROR(m) (GetLogger()->Error(__FILENAME__, __LINE__, m))

#define ASSERT(x) if (!(x)) { GetLogger()->Fatal(#x, __FILENAME__, __LINE__, nullptr); }
#define ASSERTM(x, m) if (!(x)) { GetLogger()->Fatal(#x, __FILENAME__, __LINE__, m); }

template<typename T> T *ASSERT_NOT_NULL(T * const ptr) {
  ASSERT(ptr != nullptr);
  return ptr;
}

#endif  // LOGGER_INTERFACE_
#ifndef STATUS_OR_INCLUDED_
#define STATUS_OR_INCLUDED_

#include "logger_interface.h"

enum Status {
  kSuccess, kUnavailableError, kMalformedError, kExistsError, kDoesNotExistError
};

template<typename ValueType> class StatusOr {
public:
  StatusOr(const ValueType &v) : status_(kSuccess), value_(v) {}  // Selected for lvalues.
  StatusOr(ValueType &&v) : status_(kSuccess), value_(v) {} // Selected for rvalues.
  StatusOr(Status e) : status_(e) {}
  StatusOr() : status_(kUnavailableError) {}
  
  ValueType *operator->() { ASSERT(ok()); return &value_; }
  ValueType &operator*() { ASSERT(ok()); return value_; }
  
  const ValueType *operator->() const { ASSERT(ok()); return &value_; }
  const ValueType &operator*() const { ASSERT(ok()); return value_; }

  bool ok() const { return status_ == kSuccess; }
  Status status() const { return status_; }

private:
  Status status_;
  ValueType value_;
};

#endif  // STATUS_OR_INCLUDED_
#ifndef STATUS_OR_INCLUDED_
#define STATUS_OR_INCLUDED_

enum Status {
  kSuccess, kUnavailableError, kMalformedError, kExistsError, kDoesNotExistError
};

template<typename ValueType> class StatusOr {
public:
  StatusOr(const ValueType &v) : status_(kSuccess), value_(v) {}  // Selected for lvalues.
  StatusOr(ValueType &&v) : status_(kSuccess), value_(v) {} // Selected for rvalues.
  StatusOr(Status e) : status_(e) {}
  StatusOr() : status_(kUnavailableError) {}
  
  ValueType *operator->() { return &value_; }
  ValueType &operator*() { return value_; }
  
  const ValueType *operator->() const { return &value_; }
  const ValueType &operator*() const { return value_; }

  bool ok() const { return status_ == kSuccess; }
  Status status() const { return status_; }

private:
  Status status_;
  ValueType value_;
};

#endif  // STATUS_OR_INCLUDED_
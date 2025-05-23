#ifndef STORE_INCLUDED_
#define STORE_INCLUDED_

#include "status_or.h"
#include "logger_interface.h"

// A store of optional elements.
// Store slots can be accessed by index, and may or not be set, in which case accessors
// return Status::kUnavailableError.
template<typename T, int Capacity>
class Store {
public:
  int capacity() const { return Capacity; }

  const StatusOr<T> &operator[](int index) const { 
    if (index < 0 || index >= Capacity) {
      return does_not_exist_status_;
    }
    return elements_[index]; 
  }
  // The returned reference may be mutated by other callers, regardless of whether it
  // points to an element or a Status. Callers should not rely on reading the same values
  // after control leaves their scope.
  StatusOr<T> &operator[](int index) { 
    if (index < 0 || index >= Capacity) {
      does_not_exist_status_ = Status::kDoesNotExistError;
      return does_not_exist_status_;
    }
    return elements_[index];
  }

  void Erase(int index) { elements_[index] = Status::kUnavailableError; }
  void EraseAll() { for (int i = 0; i < Capacity; ++i) { Erase(i); } }
  bool HasElement(int index) { return elements_[index].ok(); }

private:
  StatusOr<T> elements_[Capacity];
  StatusOr<T> does_not_exist_status_;
};

#endif  // STORE_INCLUDED_
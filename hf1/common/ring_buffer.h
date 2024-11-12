#ifndef RING_BUFFER__
#define RING_BUFFER__

#include <stddef.h>
#include "utils.h"
#include "logger_interface.h"

// A zero-copy ring buffer.
// Values are read and written in place.
// The newest value is always reserved for writing, so the maximum number of values the
// buffer can store is kCapacity - 1.
template<typename ValueType, int kCapacity> class RingBuffer {
  public:
    static_assert(kCapacity > 1);

    RingBuffer() { Clear(); }

    inline int Capacity() const {
      return kCapacity;
    }

    // Returns the number of values that can be read from the buffer.
    // When the buffer full, this returns kCapacity - 1, as one value is always reserved
    // for writing.
    inline int Size() const {
      return size_;
    }
        
    bool IsFull() const {
      return size_ >= kCapacity - 1;
    }

    int NumAvailableSlots() const {
      return Capacity() - 1 - Size();
    }

    // Empties the buffer.
    // Invalidates pointers obtained with OldestValue() and NewValue().
    void Clear() {
      read_index_ = 0;
      write_index_ = 0;
      size_ = 0;
      for (int i = 0; i < kCapacity; ++i) {
        indices_[i] = i;
      }
    }

    // Returns a pointer to the i-th oldest value in the buffer, or NULL if there are not enough
    // elements in the buffer.
    // The caller must consider that the pointee may mutate if someone edits the memory
    // returned by NewValue(). If an IRQ call does that, the caller should either copy the 
    // value or extend mutual exclusion throughout value access.
    // This function does not block. 
    ValueType *OldestValue(int i = 0) {
      if (Size() <= i) { return NULL; }
      return &values_[indices_[(read_index_ + i) % kCapacity]];
    }
    const ValueType *OldestValue(int i = 0) const {
      if (Size() <= i) { return NULL; }
      return &values_[indices_[(read_index_ + i) % kCapacity]];
    }
    
    // Discards the i-th oldest value in the buffer. Returns true if success, or false if
    // there is no such value to consume.
    // Invalidates the pointer obtained with OldestValue().
    bool Consume(int i = 0) {
      if (Size() <= i) { return false; }
      // Move indices one position to the right up to i.
      for (int k = 0, j = (read_index_ + i) % kCapacity; k < i; ++k, j = IndexMod(j - 1, kCapacity)) {
        indices_[j] = indices_[IndexMod(j - 1, kCapacity)];
      }
      IncReadIndex();
      --size_;
      return true;
    }

    // Returns a writable reference to a new value in the buffer. 
    // The value may be edited, but it won't be visible in OldestValue() or Size() until Commit() is called.
    // When the buffer is full, it returns a reference to the oldest value in the buffer.
    ValueType &NewValue() {
      return values_[indices_[write_index_]];
    }

    // Makes the the newest value visible to readers.
    void Commit() {
      IncWriteIndex();
      if (size_ < kCapacity - 1) {
        ++size_;
      }
      if (write_index_ == read_index_) {
        // Claim oldest unread slot for writing
        IncReadIndex();
      }
    }

    // Writes a new value in the buffer. 
    void Write(const ValueType &value) {
      NewValue() = value;
      Commit();
    }

    const ValueType Read() {
      ASSERT(OldestValue() != nullptr);
      const ValueType value = *OldestValue();
      Consume();
      return value;
    }

  protected:
    inline void IncReadIndex() {
      read_index_ = (read_index_ + 1) % kCapacity;
    }

    inline void IncWriteIndex() {
      write_index_ = (write_index_ + 1) % kCapacity;
    }

  private:
    ValueType values_[kCapacity];
    int indices_[kCapacity];
    int read_index_;
    int write_index_;
    volatile int size_;
};

#endif  // RING_BUFFER__
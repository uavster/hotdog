#ifndef RING_BUFFER__
#define RING_BUFFER__

#include <stddef.h>

template<typename ValueType, int kCapacity> class RingBuffer {
  public:
    RingBuffer() : read_index_(0), write_index_(0), size_(0) {}

    inline int Capacity() const {
      return kCapacity;
    }

    // Returns the number of values in the buffer.
    inline int Size() const {
      return size_;
    }

    // Returns a pointer to the oldest value in the buffer, or NULL if the buffer is empty.
    // The caller must consider that the pointee may mutate if someone edits the memory
    // returned by NewValue(). If an IRQ calls does that, the caller should either copy the 
    // value or extend mutual exclusion throughout value access.
    // This function does not block. 
    ValueType *OldestValue() {
      if (Size() == 0) return NULL;
      return &values_[read_index_];
    }
    const ValueType *OldestValue() const {
      if (Size() == 0) return NULL;
      return &values_[read_index_];
    }

    // Discards the oldest value in the buffer. Returns true is success, or false if there is
    // no value to consume.
    bool Consume() {
      if (Size() == 0) return false;
      IncReadIndex();
      --size_;
      return true;
    }

    // Returns a writable reference to a new value in the buffer. 
    // The value may be edited, but it won't be visible in OldestValue() or Size() until Commit() is called.
    // When the buffer is full, it returns a reference to the oldest value in the buffer.
    ValueType &NewValue() {
      return values_[write_index_];
    }

    // Makes the the newest value visible to readers.
    void Commit() {
      IncWriteIndex();
      ++size_;
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
    int read_index_;
    int write_index_;
    volatile int size_;
};

#endif  // RING_BUFFER__
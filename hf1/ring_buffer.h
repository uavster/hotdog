template<typename ValueType, int kCapacity> class RingBuffer {
  private:
    ValueType values_[kCapacity];
    int read_index_;
    int write_index_;
    volatile int size_;

  public:
    RingBuffer() : read_index_(0), write_index_(0) {
    }

    inline int Capacity() const {
      return kCapacity;
    }

    inline void IncReadIndex() {
      read_index_ = (read_index_ + 1) % kCapacity;
    }

    inline void IncWriteIndex() {
      write_index_ = (write_index_ + 1) % kCapacity;
    }

    // Returns the number of values in the buffer.
    inline int Size() {
      return size_;
    }

    // Returns the oldest value in the buffer. This function blocks until a
    // a value becomes available. It should not be called from an ISR.
    ValueType Read() {
      while (Size() == 0) {}
      NVIC_DISABLE_IRQ(IRQ_FTM0);
      ValueType value = values_[read_index_];
      IncReadIndex();
      size_--;
      NVIC_ENABLE_IRQ(IRQ_FTM0);
      return value;
    }

    // Writes a new value in the buffer. The interrupt is disabled while
    // writing the event, so this function can be called from inside or
    // outside the ISR.
    void Write(const ValueType &value) {
      NVIC_DISABLE_IRQ(IRQ_FTM0);
      values_[write_index_] = value;
      IncWriteIndex();
      size_++;
      if (write_index_ == read_index_) {
        // Claim oldest unread slot for writing
        IncReadIndex();
      }
      NVIC_ENABLE_IRQ(IRQ_FTM0);
    }
};
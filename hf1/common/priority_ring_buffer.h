#ifndef PRIORITY_RING_BUFFER_
#define PRIORITY_RING_BUFFER_

#include "ring_buffer.h"

template<typename ValueType, int kCapacity, typename PriorityType> class PriorityRingBuffer {
public:
  bool IsFull(PriorityType priority) const {
    return buffer_[priority].IsFull();
  }

  int NumAvailableSlots(PriorityType priority) const {
    return buffer_[priority].NumAvailableSlots();
  }

  ValueType *OldestValue() {
    for (int i = 0; i < PriorityType::kNumLevels; ++i) {
      ValueType *value = buffer_[i].OldestValue();
      if (value != NULL) {
        return value;
      }
    }
    return NULL;
  }

  const ValueType *OldestValue() const {
    for (int i = 0; i < PriorityType::kNumLevels; ++i) {
      const ValueType *value = buffer_[i].OldestValue();
      if (value != NULL) {
        return value;
      }
    }
    return NULL;
  }

  const ValueType *OldestValue(PriorityType priority, int i = 0) const {
    return buffer_[priority].OldestValue(i);
  }
  ValueType *OldestValue(PriorityType priority, int i = 0) {
    return buffer_[priority].OldestValue(i);
  }

  bool Consume(PriorityType priority, int i = 0) {
    return buffer_[priority].Consume(i);
  }

  ValueType &NewValue(PriorityType priority) {
    return buffer_[priority].NewValue();
  }
  
  void Commit(PriorityType priority) {
    buffer_[static_cast<int>(priority)].Commit();
  }

  int Size(PriorityType priority) const {
    return buffer_[priority].Size();
  }

  int Capacity(PriorityType priority) const {
    return kCapacity;
  }

  void Clear(PriorityType priority) const {
    buffer_[priority].Clear();
  }

  void Clear() {
    for (int i = 0; i < PriorityType::kNumLevels; ++i) {
      buffer_[i].Clear();
    }
  }

private:
  RingBuffer<ValueType, kCapacity> buffer_[PriorityType::kNumLevels];
};

#endif  // PRIORITY_RING_BUFFER_
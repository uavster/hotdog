#include "ring_buffer.h"

template<typename ValueType, int kCapacity, typename PriorityType> class PriorityRingBuffer {
public:
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

  const ValueType *OldestValue(PriorityType priority, int i = 0) {
    return buffer_[priority].OldestValue(i);
  }

  bool Consume(PriorityType priority) {
    return buffer_[priority].Consume();
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
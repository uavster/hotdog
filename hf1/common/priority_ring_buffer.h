#include "ring_buffer.h"

template<typename ValueType, int kCapacity, typename PriorityType> class PriorityRingBuffer {
public:
  const ValueType *OldestValue() const {    
    for (int i = 0; i < PriorityType::kNumLevels; ++i) {
      const ValueType *value = buffer_[i].OldestValue();
      if (value != NULL) {
        return value;
      }
    }
    return NULL;
  }

  bool Consume() {
    for (int i = 0; i < PriorityType::kNumLevels; ++i) {
      const ValueType *value = buffer_[i].OldestValue();
      if (value != NULL) {
        return buffer_[i].Consume();
      }
    }
    return false;
  }

  ValueType &NewValue(PriorityType priority) {
    return buffer_[priority].NewValue();
  }
  
  void Commit(PriorityType priority) {
    return buffer_[priority].Commit();
  }

  int Size(PriorityType priority) const {
    return buffer_[priority].Size();
  }

  int Capacity(PriorityType priority) const {
    return kCapacity;
  }

private:
  RingBuffer<ValueType, kCapacity> buffer_[PriorityType::kNumLevels];
};
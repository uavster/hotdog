template<typename ObjectT> 
StatusOr<ObjectT> PersistentStorage::get(int address) {
  ObjectT object;
  if (eeprom_.readBlock(address, reinterpret_cast<uint8_t *>(&object), sizeof(ObjectT)) != sizeof(ObjectT)) {
    return Status::kUnavailableError;
  }
  return object;
}

template<typename ObjectT> 
Status PersistentStorage::put(int address, const ObjectT &object) {
  if (eeprom_.writeBlock(address, reinterpret_cast<const uint8_t *>(&object), sizeof(ObjectT)) != 0) {
    return Status::kUnavailableError;
  }
  return Status::kSuccess;
}

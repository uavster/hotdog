#include "persistance.h"
#include "logger_interface.h"

void PersistentStorage::Init() {
  ASSERT(eeprom_.begin());
}

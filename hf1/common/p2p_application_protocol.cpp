#include "p2p_application_protocol.h"
#include "logger_interface.h"

const char *GetTrajectoryViewTypeName(P2PTrajectoryViewType type) {
  static char name_map[kNumTrajectoryViewTypes][10] = { "plain", "modulated", "mixed" };
  const size_t index = static_cast<size_t>(type);
  ASSERT(index < sizeof(name_map) / sizeof(name_map[0]));
  return name_map[index];
}
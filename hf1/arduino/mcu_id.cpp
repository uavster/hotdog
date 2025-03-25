#include "mcu_id.h"

MCUID::MCUID() {
  const uint32_t sim_sdid = SIM_SDID;
  family_id = (sim_sdid >> 4) & 0x7;
  revision_id = (sim_sdid >> 12) & 0xf;
  pin_id = sim_sdid & 0xf;
}

void MCUID::Print(Stream &stream) {
  stream.printf("MCU family: ");
  if (family_id >= 0b000 && family_id <= 0b011) { 
    stream.printf("K%d", 10 * (family_id + 1));
  } else if (family_id >= 0b110 && family_id <= 0b111) {
    stream.printf("K%d", 50 + family_id - 0b110);
  } else {
    stream.print("Unknown");
  }
  stream.println();
  stream.printf("MCU revision ID: %d\n", revision_id);  
  stream.printf("MCU pin count: ");
  switch(pin_id) {
    case 0b0101:
      stream.print("64");
      break;
    case 0b0110:
      stream.print("80");
      break;
    case 0b0111:
      stream.print("81");
      break;
    case 0b1000:
      stream.print("100");
      break;
  }
  stream.println();  
}

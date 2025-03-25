#ifndef MCU_ID_INCLUDED_
#define MCU_ID_INCLUDED_

#include "null_stream.h"

class MCUID {
public:
  int family_id;
  int revision_id;
  int pin_id;

  MCUID();
  void Print(Stream &stream = null_stream);
};

#endif


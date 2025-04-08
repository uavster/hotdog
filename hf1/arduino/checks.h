#include <Arduino.h>
#include "null_stream.h"

bool CheckMCU(Stream &stream = null_stream);
bool CheckSRAM(Stream &stream = null_stream);
bool CheckBattery(Stream &stream = null_stream);
bool CheckEEPROM(Stream &stream = null_stream);
bool CheckTimer(Stream &stream = null_stream);
bool CheckMotors(Stream &stream = null_stream, bool check_preconditions = false);
bool CheckEncoders(Stream &stream, bool check_preconditions = false);
bool CheckBodyIMU(Stream &stream, bool check_preconditions = false);
bool CheckBodyMotion(Stream &stream, bool check_preconditions = false);
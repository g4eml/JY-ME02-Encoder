/*
  EncoderCommands.h - simple text commands for a JYME02 encoder.

  Commands (case-insensitive, end with newline - a command with no line
  ending is also accepted after a short pause):
    Z           - zero (current position becomes 0 deg)
    S <angle>   - set current position, e.g. "S 271.20" or "S -15"

  Call encoderCommandsUpdate(encoder, Serial) from loop().
*/
#pragma once
#include <Arduino.h>
#include "JYME02.h"

void encoderCommandsUpdate(JYME02 &encoder, Stream &io);

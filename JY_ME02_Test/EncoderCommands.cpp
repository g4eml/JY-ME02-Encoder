/*
  EncoderCommands.cpp - see EncoderCommands.h
*/
#include "EncoderCommands.h"
#include <ctype.h>
#include <stdlib.h>

static void runCommand(JYME02 &encoder, Stream &io, char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  char  cmd = toupper(*line);
  char *arg = line + 1;

  switch (cmd) {
    case 'Z':
      io.println("# zeroing encoder...");
      io.println(encoder.zero() ? "# zero done" : "# zero FAILED");
      break;

    case 'S': {
      char *end;
      double value = strtod(arg, &end);
      if (end == arg) {
        io.println("# usage: S <angle>   e.g. S 271.20");
        break;
      }
      io.print("# setting angle to ");
      io.println(value, 3);
      io.println(encoder.setAngle(value) ? "# set done" : "# set FAILED");
      break;
    }

    default:
      io.print("# unknown command: ");
      io.println(line);
      break;
  }
}

void encoderCommandsUpdate(JYME02 &encoder, Stream &io) {
  static char     buf[32];
  static size_t   len = 0;
  static uint32_t lastCharMs = 0;

  while (io.available()) {
    char c = io.read();
    lastCharMs = millis();
    if (c == '\r' || c == '\n') {
      buf[len] = '\0';
      runCommand(encoder, io, buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }

  // No line ending received: run the command after a short pause.
  if (len > 0 && millis() - lastCharMs > 250) {
    buf[len] = '\0';
    runCommand(encoder, io, buf);
    len = 0;
  }
}

/*
  Antenna rotator position from a JY-ME02-485 encoder - RP2040 (arduino-pico)

  Tabs:
    JYME02.h / .cpp          - encoder driver (reusable)
    EncoderCommands.h / .cpp - USB serial commands Z and S <angle> (reusable)

  Wiring: RP2040 GP0 (TX) -> RS-485 converter DI, GP1 (RX) <- RO,
          converter A/B -> encoder YELLOW/GREEN, common GND.

  USB output (CSV): antenna_deg,antenna_deg_per_s
*/
#include "JYME02.h"
#include "EncoderCommands.h"

// ---- project settings ----
const float ANTENNA_DEG_PER_SHAFT_REV = 6.0;   // antenna deg per encoder turn

JYME02 encoder(Serial1);                       // Serial1, ID 0x50, no DE pin

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

  encoder.setGearRatio(ANTENNA_DEG_PER_SHAFT_REV);
  encoder.setDebug(Serial);
  encoder.begin(9600, 0, 1);                   // baud, TX pin, RX pin

  Serial.println("antenna_deg,antenna_deg_per_s");
}

void loop() {
  encoderCommandsUpdate(encoder, Serial);

  if (encoder.update()) {
    Serial.print(encoder.getAngle(), 3);
    Serial.print(',');
    Serial.println(encoder.getVelocity(), 3);
  }
}

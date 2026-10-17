/*
  JYME02.h - driver for the WitMotion JY-ME02-485 absolute encoder
  (Modbus RTU over RS-485), written for the RP2040 arduino-pico core.

  The encoder can drive an output through a gear ratio (e.g. an antenna
  rotator). All angles/velocities returned are OUTPUT angles, scaled by
  setGearRatio(). With the default ratio (360 deg per shaft rev) they are
  simply the encoder shaft values.

  Typical use:
      JYME02 encoder(Serial1);               // port, optional ID / DE pin
      encoder.setGearRatio(6.0);             // output deg per shaft rev
      encoder.begin(9600, 0, 1);             // baud, TX pin, RX pin
      ...
      if (encoder.update()) {                // true when new data read
          double a = encoder.getAngle();     // deg
          float  v = encoder.getVelocity();  // deg/s
      }

  Register / protocol notes (manual chapter 6/7):
    0x00 SAVE, 0x10 ENCODER_MODE, 0x11 ANG_VAL, 0x12 REVOLUTI,
    0x13 ANGSPE_VAL, 0x17 ANGSPE_SAMPLING_TIME, 0x69 unlock (0xB588).
    The manual labels the checksum "CRCH CRCL" but the device uses standard
    Modbus CRC-16, sent LOW byte first.
*/
#pragma once
#include <Arduino.h>

#if defined(ARDUINO_ARCH_RP2040)
  typedef SerialUART JYME02Port;      // Serial1 / Serial2 on arduino-pico
#else
  typedef HardwareSerial JYME02Port;
#endif

class JYME02 {
public:
  // port   : UART the RS-485 converter is on (e.g. Serial1)
  // id     : Modbus address (default 0x50)
  // dirPin : DE/RE pin for converters that need it, -1 for auto-direction
  JYME02(JYME02Port &port, uint8_t id = 0x50, int dirPin = -1);

  // ---- configuration (call before begin) ----
  // Output degrees per one full turn of the encoder shaft (default 360).
  // Negative reverses the direction.
  void setGearRatio(float outputDegPerShaftRev);
  // How often update() polls the encoder (default 100 ms).
  void setPollInterval(uint32_t ms);
  // Optional stream for status / error messages (lines start with '#').
  void setDebug(Stream &s);

  // Start the UART, switch the encoder to multi-turn mode if needed and
  // read its velocity sampling time. Pins < 0 keep the core's defaults.
  // Returns true if the encoder responded.
  bool begin(uint32_t baud = 9600, int txPin = -1, int rxPin = -1);

  // ---- reading ----
  // Call often from loop(). Polls the encoder every poll interval.
  // Returns true when a new valid reading has just been taken.
  bool update();
  // Poll immediately, ignoring the interval. Returns true on success.
  bool read();

  double getAngle() const;         // output angle, continuous (e.g. -20, 400)
  double getAngleWrapped() const;  // output angle wrapped to 0..360
  float  getVelocity() const;      // output velocity, deg/s (+/- direction)

  double  getShaftAngle() const;   // raw shaft angle 0..360
  int16_t getRevolutions() const;  // raw shaft revolution count

  bool     isOk() const;           // true if the last poll succeeded
  uint32_t lastReadMs() const;     // millis() of the last good reading

  // ---- setting position (keep the shaft still while these run) ----
  bool setAngle(double outputDeg); // make current position read as outputDeg
  bool zero();                     // same as setAngle(0)

private:
  JYME02Port &_port;
  uint8_t  _id;
  int      _dirPin;
  float    _ratio      = 360.0f;
  uint32_t _pollMs     = 100;
  Stream  *_debug      = nullptr;
  float    _sampleTimeS = 0.1f;

  uint16_t _angRaw   = 0;
  int16_t  _revs     = 0;
  int16_t  _speedRaw = 0;
  bool     _ok       = false;
  uint32_t _lastPollMs = 0;
  uint32_t _lastGoodMs = 0;

  void log(const char *msg);
  uint16_t crc16(const uint8_t *buf, size_t len);
  void buildFrame(uint8_t *f, uint8_t func, uint16_t a, uint16_t b);
  void transmit(const uint8_t *buf, size_t len);
  bool receive(uint8_t *buf, size_t len);
  bool readRegisters(uint16_t reg, uint16_t count, uint16_t *out);
  bool writeRegister(uint16_t reg, uint16_t value);
  bool writeSetting(uint16_t reg, uint16_t value);
  bool ensureMultiTurn();
  bool readSampleTime();
};

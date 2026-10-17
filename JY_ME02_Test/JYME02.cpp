/*
  JYME02.cpp - driver for the WitMotion JY-ME02-485 absolute encoder.
  See JYME02.h for usage.
*/
#include "JYME02.h"
#include <math.h>
#include <string.h>

// Registers
static const uint16_t REG_SAVE        = 0x00;
static const uint16_t REG_MODE        = 0x10;
static const uint16_t REG_ANG_VAL     = 0x11;
static const uint16_t REG_REVOLUTI    = 0x12;
static const uint16_t REG_SAMPLE_TIME = 0x17;
static const uint16_t REG_UNLOCK      = 0x69;
static const uint16_t UNLOCK_VALUE    = 0xB588;
static const uint16_t SAVE_CURRENT    = 0x0000;
static const uint16_t MODE_MULTI_TURN = 0x0001;

static const float    COUNTS_PER_REV      = 32768.0f;
static const uint32_t RESPONSE_TIMEOUT_MS = 100;
static const uint32_t WRITE_GAP_MS        = 20;

JYME02::JYME02(JYME02Port &port, uint8_t id, int dirPin)
  : _port(port), _id(id), _dirPin(dirPin) {}

void JYME02::setGearRatio(float outputDegPerShaftRev) { _ratio = outputDegPerShaftRev; }
void JYME02::setPollInterval(uint32_t ms)             { _pollMs = ms; }
void JYME02::setDebug(Stream &s)                      { _debug = &s; }

void JYME02::log(const char *msg) {
  if (_debug) { _debug->print("# "); _debug->println(msg); }
}

// ---------------- start-up ----------------
bool JYME02::begin(uint32_t baud, int txPin, int rxPin) {
  if (_dirPin >= 0) {
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_dirPin, LOW);            // receive mode
  }
#if defined(ARDUINO_ARCH_RP2040)
  if (txPin >= 0) _port.setTX(txPin);
  if (rxPin >= 0) _port.setRX(rxPin);
#else
  (void)txPin; (void)rxPin;
#endif
  _port.begin(baud, SERIAL_8N1);

  delay(200);                              // let the encoder finish booting
  bool ok = ensureMultiTurn();
  ok = readSampleTime() && ok;
  return ok;
}

// Revolutions are only counted in multi-turn mode.
bool JYME02::ensureMultiTurn() {
  uint16_t mode;
  if (!readRegisters(REG_MODE, 1, &mode)) {
    log("could not read encoder mode");
    return false;
  }
  if (mode == MODE_MULTI_TURN) return true;

  log("encoder in single-turn mode - switching to multi-turn");
  bool ok = writeSetting(REG_MODE, MODE_MULTI_TURN) &&
            writeSetting(REG_SAVE, SAVE_CURRENT);
  log(ok ? "multi-turn mode set" : "FAILED to set multi-turn mode");
  return ok;
}

// Velocity scaling depends on the sampling time (value * 100 us).
bool JYME02::readSampleTime() {
  uint16_t raw;
  if (readRegisters(REG_SAMPLE_TIME, 1, &raw) && raw > 0) {
    _sampleTimeS = raw * 100e-6f;
    return true;
  }
  log("could not read sampling time - assuming 100 ms");
  return false;
}

// ---------------- reading ----------------
bool JYME02::update() {
  if (millis() - _lastPollMs < _pollMs) return false;
  return read();
}

bool JYME02::read() {
  _lastPollMs = millis();
  uint16_t regs[3];                        // ANG_VAL, REVOLUTI, ANGSPE_VAL
  _ok = readRegisters(REG_ANG_VAL, 3, regs);
  if (!_ok) {
    log("no/invalid response from encoder");
    return false;
  }
  _angRaw     = regs[0];
  _revs       = (int16_t)regs[1];
  _speedRaw   = (int16_t)regs[2];
  _lastGoodMs = _lastPollMs;
  return true;
}

double JYME02::getAngle() const {
  double shaftRevs = _revs + _angRaw / (double)COUNTS_PER_REV;
  return shaftRevs * _ratio;
}

double JYME02::getAngleWrapped() const {
  double a = fmod(getAngle(), 360.0);
  return (a < 0) ? a + 360.0 : a;
}

float JYME02::getVelocity() const {
  float shaftDegPerS = _speedRaw * 360.0f / COUNTS_PER_REV / _sampleTimeS;
  return shaftDegPerS * _ratio / 360.0f;
}

double   JYME02::getShaftAngle()  const { return _angRaw * 360.0 / COUNTS_PER_REV; }
int16_t  JYME02::getRevolutions() const { return _revs; }
bool     JYME02::isOk()           const { return _ok; }
uint32_t JYME02::lastReadMs()     const { return _lastGoodMs; }

// ---------------- setting position ----------------
bool JYME02::setAngle(double outputDeg) {
  double shaftRevs = outputDeg / _ratio;
  double whole     = floor(shaftRevs);
  long   counts    = lround((shaftRevs - whole) * COUNTS_PER_REV);
  if (counts >= (long)COUNTS_PER_REV) {    // rounding pushed onto next rev
    counts -= (long)COUNTS_PER_REV;
    whole  += 1.0;
  }
  if (whole < -32768.0 || whole > 32767.0) {
    log("angle out of range for the revolution counter");
    return false;
  }

  bool ok = writeSetting(REG_ANG_VAL, (uint16_t)counts) &&
            writeSetting(REG_REVOLUTI, (uint16_t)(int16_t)whole) &&
            writeSetting(REG_SAVE, SAVE_CURRENT);
  if (ok) read();                          // refresh cached values
  return ok;
}

bool JYME02::zero() { return setAngle(0.0); }

// ---------------- Modbus RTU ----------------
uint16_t JYME02::crc16(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

void JYME02::buildFrame(uint8_t *f, uint8_t func, uint16_t a, uint16_t b) {
  f[0] = _id;     f[1] = func;
  f[2] = a >> 8;  f[3] = a & 0xFF;
  f[4] = b >> 8;  f[5] = b & 0xFF;
  uint16_t crc = crc16(f, 6);
  f[6] = crc & 0xFF;                       // CRC low byte first
  f[7] = crc >> 8;
}

void JYME02::transmit(const uint8_t *buf, size_t len) {
  while (_port.available()) _port.read();  // discard stale bytes
  if (_dirPin >= 0) digitalWrite(_dirPin, HIGH);
  _port.write(buf, len);
  _port.flush();
  if (_dirPin >= 0) digitalWrite(_dirPin, LOW);
}

bool JYME02::receive(uint8_t *buf, size_t len) {
  size_t got = 0;
  uint32_t start = millis();
  while (got < len && millis() - start < RESPONSE_TIMEOUT_MS) {
    if (_port.available()) buf[got++] = _port.read();
  }
  return got == len;
}

bool JYME02::readRegisters(uint16_t reg, uint16_t count, uint16_t *out) {
  uint8_t req[8];
  buildFrame(req, 0x03, reg, count);
  transmit(req, sizeof(req));

  const size_t expected = 5 + 2 * count;   // ID 03 N data.. CRC_L CRC_H
  uint8_t resp[5 + 2 * 8];
  if (expected > sizeof(resp)) return false;
  if (!receive(resp, expected)) return false;
  if (resp[0] != _id || resp[1] != 0x03 || resp[2] != 2 * count) return false;
  uint16_t rxCrc = resp[expected - 2] | (resp[expected - 1] << 8);
  if (rxCrc != crc16(resp, expected - 2)) return false;

  for (uint16_t i = 0; i < count; i++) out[i] = (resp[3 + 2 * i] << 8) | resp[4 + 2 * i];
  return true;
}

bool JYME02::writeRegister(uint16_t reg, uint16_t value) {
  uint8_t req[8], resp[8];
  buildFrame(req, 0x06, reg, value);
  transmit(req, sizeof(req));
  if (!receive(resp, sizeof(resp))) return false;
  return memcmp(req, resp, sizeof(req)) == 0;   // device echoes the write
}

// Every setting must be preceded by the unlock command.
bool JYME02::writeSetting(uint16_t reg, uint16_t value) {
  if (!writeRegister(REG_UNLOCK, UNLOCK_VALUE)) return false;
  delay(WRITE_GAP_MS);
  bool ok = writeRegister(reg, value);
  delay(WRITE_GAP_MS);
  return ok;
}

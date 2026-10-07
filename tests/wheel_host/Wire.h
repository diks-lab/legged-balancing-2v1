#pragma once
#include "Arduino.h"
class TwoWire {
 public:
  explicit TwoWire(int) {}
  bool busOK = true;
  uint8_t nack = 0, count = 3, magnet = 0x20;
  uint16_t raw = 0;
  int index = 0;
  uint32_t latency = 0;
  std::function<void()> onRequest;
  bool begin(int, int, int) { return busOK; }
  void setTimeOut(uint16_t) {}
  void beginTransmission(uint8_t) {}
  void write(uint8_t) {}
  uint8_t endTransmission(bool) { delay(latency); return nack; }
  uint8_t requestFrom(uint8_t, uint8_t) { if (onRequest) onRequest(); index = 0; return count; }
  int available() { return count - index; }
  int read() { int values[] = {magnet, raw >> 8, raw & 255}; return values[index++]; }
};

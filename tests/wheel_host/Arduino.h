#pragma once
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>
#include <deque>
#include <vector>
#include <utility>
#include <functional>
#include <initializer_list>
using std::isfinite;
constexpr int LOW = 0, HIGH = 1, OUTPUT = 1;
inline uint32_t nowMs = 0;
inline int pins[64] = {};
inline std::vector<std::pair<int, int>> gpioEvents;
inline std::function<void()> tickTimer;
inline void digitalWrite(int pin, int value) { pins[pin] = value; gpioEvents.emplace_back(pin, value); }
inline void pinMode(int pin, int mode) { gpioEvents.emplace_back(pin, mode + 10); }
inline int digitalRead(int pin) { return pins[pin]; }
inline uint32_t millis() { return nowMs; }
inline uint32_t micros() { return nowMs * 1000; }
inline void delay(uint32_t ms) { nowMs += ms; if (tickTimer) tickTimer(); }
struct FakeSerial {
  std::deque<unsigned char> rx;
  std::string output;
  int room = 128;
  void begin(int) {}
  int available() { return rx.size(); }
  int read() { int c = rx.front(); rx.pop_front(); return c; }
  int availableForWrite() { return room; }
  void write(uint8_t c) { output += char(c); }
  void flush() {}
  template<typename... A> void printf(const char* s, A... a) {
    char b[512]; std::snprintf(b, sizeof(b), s, a...); output += b;
  }
  void feed(const std::string& s) { for (unsigned char c : s) rx.push_back(c); }
};
inline FakeSerial Serial;
